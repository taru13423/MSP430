#!/usr/bin/python3 -u
# -*- coding: utf-8 -*-

'''
 Armadillo上のLPR9204コンセントレータ（マルチスレッド制御）

 各ノードから送られてきたデータを受信して、
 足らなければ再送要求して、
 データがそろったら全ノードを60-かかった秒だけ寝かせる
'''

import serial
import sys
import codecs
import math
import os
import threading
import subprocess
import pyfiap
import linecache
from datetime import datetime
from collections import deque
from time import sleep

NODE_LIST_FILE = '/home/tarutani/node_list.txt'
PID_FILE = '/var/run/concentratord.pid'
OUTPUT_FILE = '/home/tarutani/temp_humi_'
INTERMITTENT_TIME = 60

serial_readline = deque() # シリアルデータ
received_packets = deque() # パケット保存用キュー
accepted_packets = [] # 受理したパケット保存用リスト
coordinate = {} # 各ノードの座標保管用ディクショナリ (16進4桁ノードID, x座標, y座標)
ack_message_queue = deque() # EACKメッセージ保存用キュー
message_id_queue = deque()  # 920が割り振るメッセージID保存用キュー
error_code_queue = deque()  # FAIL ER10などのエラーコード保存用キュー

s = serial.Serial('/dev/ttymxc1', 9600, timeout=1)
#s = serial.Serial('/dev/tty.usbserial-AL00MUQ0', 115200, timeout=1)
#s = serial.Serial('COM3', 115200, timeout=1)
packet_number = 0
fiap = pyfiap.fiap.APP("http://ants.jga.kisarazu.ac.jp/axis2/services/FIAPStorage?wsdl")

'''
 パケットを受け取ることに専念するスレッド
 ERXDATA_0000_0000_0000_0000_00_00_00000_0000_0000
         FROM DEST MID  SEL  RS LEN DATA RUT1 RUT2
'''

def receive_packet(e, ):
    while not e.isSet():
        if( s.readable() and s.in_waiting > 0 ):
            data = s.read(s.in_waiting).decode('utf-8') #ここで受け取ってる
            print("Serial>"+data)
            if(len(data)>0):
                serial_readline.append(data)
                sksend_sleep(data)

def sksend_sleep(data):
    global sleep_number
    data_list = data.split()
    if( len(data_list)>2 and data_list[0] == "ERXDATA"):
        sleep_time = INTERMITTENT_TIME - datetime.now().second
        if(sleep_time >= 60):
            sleep_time = 59
        elif(sleep_time < 10):
            sleep_time += 60
        send_packet("SKSEND 1 1000 "+data_list[1]+" 0F SLEEP,"+data_list[1]+",0,"+str(sleep_time))
        print('sleep_time>{0} now.second >> {1}'.format(sleep_time,datetime.now().second))
        sleep(1)
        send_packet("SKSEND 1 1000 "+data_list[1]+" 0F SLEEP,"+data_list[1]+",0,"+str(sleep_time))

def routing_packet(e, ):
    line_buffer = ""
    while not e.isSet():
        if( len(serial_readline) > 0 ):
            line_buffer = line_buffer + serial_readline.popleft()
            if( '\r\n' in line_buffer ):
                lines = line_buffer.split('\r\n')
                line_buffer = lines[len(lines)-1]
                lines[len(lines)-1] = ""
                for line in lines:
                    if( len(line) > 1 ):
                        print("   920>"+ line)
                        push_each_queue(line)

def push_each_queue( line ):
    data = line.rstrip().split()
    if( len(data)>=8 and data[0] == 'ERXDATA' ):
        t_and_h = data[7].split(',')
        if( len(t_and_h) >= 3 ):
            send_id = data[1]
            p_id   = int(t_and_h[0]) # packet_id
            temp   = t_and_h[1]
            humi   = t_and_h[2]
            packet = (send_id, p_id, temp, humi)
            received_packets.append(packet)

            now  = datetime.now()
            day  = now.strftime("%Y%m%d")
            date = now.strftime("%H:%M:%S")
            sleep_time = INTERMITTENT_TIME - now.second
            if(sleep_time >= 60):
                sleep_time = 59
            elif(sleep_time < 10):
                sleep_time += 60

            svp = temp2svp(float(temp))  # Saturated Vapor Pressure [Pa]
            vp = svp * float(humi) / 100 # Vapor Pressure [Pa]
            vpd = (svp-vp)/1000   # Vapour Pressure Dificit [kPa]
            csv = date+'\t'+str(int(sleep_time))+str(temp)+'\t'+str(humi)+'\t'+str(vpd)
            if(os.path.isfile(OUTPUT_FILE + send_id + '_' + day+'.csv')):

                num_lines = sum(1 for line in open(OUTPUT_FILE + send_id + '_' + day+'.csv'))
                target_line = linecache.getline(OUTPUT_FILE + send_id + '_' + day+'.csv', num_lines)
                linecache.clearcache()
                print("----------------------------------")
                print("csv[1] >> {0}, tar[1] >> {1}".format(csv.split('\t')[1], target_line.split('\t')[1]))
                print("csv[2] >> {0}, tar[2] >> {1}".format(csv.split('\t')[2], target_line.split('\t')[2]))
                print("----------------------------------")
                if (csv.split('\t')[1] == target_line.split('\t')[1] and
                csv.split('\t')[2] == target_line.split('\t')[2]):
                    print("onazi!")
                else:
                    with open(OUTPUT_FILE + send_id + '_' + day+'.csv', 'a') as f:
                        f.write(csv+'\r\n')
            else:
                with open(OUTPUT_FILE + send_id + '_' + day+'.csv', 'a') as f:
                    f.write(csv+'\r\n')

    elif( len(data)>=4 and data[0] == 'EACK' ):
        ack_result = ( data[1], data[3] ) # STATUS, MSG_ID
        ack_message_queue.append( ack_result )
    elif( len(data)>=2 and data[1] == 'OK' ):
        message_id_queue.append(data[0])
    elif( len(data)>=2 and data[0] == 'FAIL' ):
        error_code_queue.append(data[1])

'''
 受け取ったパケットを処理する(ARQ)スレッド
 @args packet_number パケット番号が異なるものは破棄する
 1. 全ノードからデータが送られてきているかをチェック
 2. 送られてきていないノードに対しては再送要求
 3. データが揃えばスレッド終了
'''

def automatic_repeat_request(e, packet_number):
    coordinate.clear() # 今回取得すべき座標一覧
    with codecs.open( NODE_LIST_FILE, 'r', 'utf-8' ) as f:
        for line in f:
            line = line.strip()
            comment = line.replace(' ','').split('#')
            if comment[0]:
                data = comment[0].split(',')
                coordinate[data[0]] = (data[1], data[2])
    start_sec = datetime.now().second
    # 座標情報
    while( len(coordinate) > 0 and not e.isSet() ):

        if( len(received_packets)>0 ):
            packet = received_packets.popleft()
            print("Packet>",end="")
            print(packet)
            # 対象のパケットIDでかつ、対象の座標である

            if( packet[1] == packet_number and packet[0] in coordinate ):
                coord = coordinate.pop(packet[0]) # 座標情報の更新
                packet += coord # パケットに座標情報を付与
                accepted_packets.append(packet)
                print("Captured: " + packet[0] + " and Remaining : ", end="" )
                print(coordinate.keys())

'''
 寝ているときに受け取ったパケットを処理する(insleep)スレッド
 @args packet_number 次のパケットのデータであれば寝かさず起こしておく
'''

def insleep_automatic_repeat_request(e_slp, packet_number):
    sec = 0
    while( e_slp.isSet() ):
        if( len(received_packets)>0 ):
            packet = received_packets.popleft()
            if( packet[1] != packet_number ): # でたらめなパケットIDのデータが届けばsleepさせる
#                sleep_time = broadcast_sleep_all(1)
                now = datetime.now().second
                if( now - sec > 2 ):
                    sec = now # 前回の実行から2秒以上経っていれば送信する
                    sleep_time = broadcast_sleep_all(1)
            else: # 次のパケットならもう一回received_packetsに入れてぐーるぐる
                received_packets.append(packet)

'''
 パケット送信（×再送制御）
 IoTノードが寝てるときに再送制御してもずっと寝てるので再送してもあまり意味は無い
 メソッド側で改行コードは面倒を見ます
'''
def send_packet( command ):
    s.flush()
    _send_packet_and_get_message_id( command )

'''
 sleep_allを発信
'''
def broadcast_sleep_all( no_resend ):
    sleep_time = INTERMITTENT_TIME - datetime.now().second
    if( sleep_time >= INTERMITTENT_TIME ):
        sleep_time = 1
    str_st = '{0:02d}'.format(sleep_time)
    # SLEEPはブロードキャスト（最大3ホップ）で送信
    broadcast_packet("SKBC 3 1000 0F SLEEP_ALL0_,"+str(packet_number)+","+str_st, no_resend)
    return(sleep_time)

'''
 ブロードキャスト送信（再送制御なし）
 メソッド側で改行コードは面倒を見ます
'''

def broadcast_packet( command, no_resend ):
    for i in range(no_resend):
        message_id = _send_packet_and_get_message_id( command )
        if( len(error_code_queue) > 0 ): # エラーメッセージがでたときは再送
            error_message = error_code_queue.popleft()
            print("[Resend because of error] "+command+" / "+error_message)
            message_id = _send_packet_and_get_message_id( command )
        sleep(1)

'''
 パケットを送信してメッセージIDを取得します
'''

def _send_packet_and_get_message_id( command ):
    serial_command = command + "\r\n"
    s.write(serial_command.encode('utf-8'))

'''
 ack_message_queueから対象のmessage_idを含むACK結果を取り出して返します
'''
def pop_ack_message( sent_message_id ):

    for ack in ack_message_queue: # ack[0] = STATUS, ack[1] = MSG_ID

        if( ack[1] in sent_message_id ):
            ack_message_queue.remove(ack)
            sent_message_id.remove(ack[1])
            return ack[0] # ack_status '0' or '1'
    return '0' # ACKが受け取れていないので'0'を返して再送

'''
 Sonntag近似式を使って飽和水蒸気圧[Pa]を求めます
'''
def temp2svp( temp ):
    temp = temp+273.15
    a = -6096.9385 / temp
    b = 16.635794
    c = -2.711193 / 100 * temp
    d = 1.673952 / 100000 * temp * temp
    e = 2.433502 * math.log(temp)
    return( math.exp( a + b + c + d + e ) * 100 )


'''
 メインスレッド
'''
def main_thread():
    global packet_number
    e_rec = threading.Event()
    e_arq = threading.Event()
    e_slp = threading.Event()
    e_rec.clear()
    th_rec = threading.Thread(name='rec', target=receive_packet, args=(e_rec,))
    th_rec.start()
    th_route = threading.Thread(name='route', target=routing_packet, args=(e_rec,))
    th_route.start()
    print("Start threading!")
    try:
        while True:
            e_arq.clear()
            th_arq = threading.Thread(name='arq', target=automatic_repeat_request, args=(e_arq, packet_number))
            th_arq.start()
            th_arq.join(30) # 30秒で打ち切る
            e_arq.set()
            print("[Information] Finish waiting ARQ thread in main.")
            e_slp.set()
            th_slp = threading.Thread(name='insleep', target=insleep_automatic_repeat_request, args=(e_slp, packet_number))
            th_slp.start()
            packet_number = ( packet_number+1 ) % 10
            e_slp.clear()


            for packet in accepted_packets:
                # packet = (send_id, packet_number, temp, humi, x_pos, y_pos)
                send_id = packet[0]
                temp = float(packet[2])
                humi = float(packet[3])
                x_pos = packet[4]
                y_pos = packet[5]
                posit = x_pos + '_' + y_pos
                print(send_id+":"+posit+", ",end="")

    except KeyboardInterrupt:
        print("W: interrupt received, stopping script")
    finally:
        print("W: close...")
        s.close()
    e_rec.set()
    e_arq.set()
    e_slp.clear()
    th_rec.join()
    sys.exit()

if __name__ == '__main__':
    main_thread()
