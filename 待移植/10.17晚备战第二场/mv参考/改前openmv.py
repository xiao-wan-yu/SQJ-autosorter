'''改前版本'''




# 机器人识别色块

# 黄=（60,90,-20,0,40,70）

# 红=(15,60,20,50,0,40)

# 蓝=（15，30，-10，20，-45，-10 ）
# 35,65,-55,-38,20,40lu

import sensor, image, time, math
import json,utime
from pyb import UART
from pyb import Pin, Servo

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=1000)
sensor.set_auto_gain(False)         #关闭自动增益
sensor.set_auto_whitebal(False)     #寻找色块不能开白平衡  #不关上述两项会有影响阈值效果
#sensor.set_vflip(True)             #垂直翻转
#sensor.set_hmirror(1)           #镜像翻转
#sensor.run(1)
#sensor.set_contrast(1)
#sensor.set_gainceiling(16)

y_thresholds  = (60, 94, -18, 7, 9, 76)  # 黄
y2_thresholds = (38, 93, 60, 7, 106, 40) # 黄亮阈值
y3_thresholds = (50, 99, 3, -31, 99, 53)
w_thresholds = (48, 100, -11, 17, -35, 0)
w1_thresholds = (92, 100, -17, 5, -7, 5)
w2_thresholds = (44, 90, -14, 14, -40, -6)
w3_thresholds = (90, 100, -37, 58, -46, 32)



r_thresholds = ((38, 64, 82, 15, 7, 43)) # 红
b_thresholds = (40, 72, -26, 21, -63, -15)  # 蓝
b2_thresholds = ((7, 67, 46, 5, -81, -16))
g_thresholds = (35, 65, -55, -38, 20, 40)  # lv
# 3，86，141，87

##阶梯阈值
rr_thresholds =(38, 46, 36, 72, 22, 66)
rr1_thresholds = (20, 46, 21, 63, 0, 45)





clock = time.clock()
ROI = (3, 87, 219, 75)

uart = UART(3, 9600)  # P4为TX P5为RX
#p_out_0 = Pin('P0', Pin.OUT_PP)


t_t = 400       #320
pia = 1600
hui = 2050
t_1 = 20
t_2 = 200
ROI1 = (40,40,80,80)
ROI2 = (45,40,80,80)
W_X = 80+1
W_Y = 60-7


i = 0

B0 = 0  # 为接收串口标志
C0 = 0  #红色方标志位
H0 = 0  #蓝色方标志位

S1 = 1

D2 = 1
D3 = 1
D1= 1

D0 = 1  # 0
E0 = 0

C1 = 0
C2 = 0
C3 = 0

BLUE = 0  # 蓝球标志位（用于计数）
RED = 0  # 红球标志位
Recognition_counts = 0 # 阶梯识别次数

W0 = 0
F0 = 0
G0 = 0
R0 = 0
J0 = 1
J1 = 0   #红
J2 = 0   #蓝

s1=Servo(1)
s2=Servo(2)
def bense():#打到右边,本色
    #s1.pulse_width(2200)
    s1.pulse_width(pia)

    time.sleep_ms(t_1)

    s2.pulse_width(2000)

    time.sleep_ms(t_2)
    s1.pulse_width(hui)
    time.sleep_ms(t_t - t_1 - t_2)

def huangse():#打到左边，黄色
    #s1.pulse_width(850)
    s1.pulse_width(pia)

    time.sleep_ms(t_1)
    s2.pulse_width(1120)
    time.sleep_ms(t_2)
    s1.pulse_width(hui)
    time.sleep_ms(t_t - t_1 - t_2)

def jiaozhun():   #中心点为(80,60)
    global D2,D3,D1
    s1.pulse_width(2200)    ##？

    print("y")
    while(D2):     ##校准y位置

        if(uart.any()):
            uart.read()
            D2_1 = 1
            while D2_1  :
                img = sensor.snapshot()
                blobs = img.find_blobs([w3_thresholds], pixels_threshold=600, area_threshold=800)
                if blobs:
                    for b in blobs:
                        img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                        img.draw_cross(b[5], b[6])  ##画十字交叉
                        #print(b[5]-W_X)

                        if b[5]-W_X>6:
                            uart.write('9')   #向前走
                            D2_1 = 0
                        elif W_X-b[5]>6:
                            uart.write('0')   #向后走
                            D2_1 = 0
                        else:
                            D2 = 0
                            D2_1 = 0
                            break
    print("x")
    uart.write('2')    ##告诉单片机y校准完成
    while(D3):        ##校准左右位置

        if (uart.any()):
            uart.read()
            print("shoudao")
            D3_1 = 1
            while D3_1 :
                img = sensor.snapshot()
                blobs = img.find_blobs([w3_thresholds], pixels_threshold=600, area_threshold=800)
                if blobs:
                    for b in blobs:
                        img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                        img.draw_cross(b[5], b[6])  ##画十字交叉

                        #print(b[6]-W_Y)
                        if b[6]-W_Y>6:
                            uart.write('7')   #向左走
                            D3_1 = 0
                        elif W_Y-b[6]>6:
                            uart.write('8')   #向右走
                            D3_1 = 0
                        else:
                            D3 = 0
                            D1 = 0
                            print("yxbreak")
                            D3_1 = 0
                            break

#调1
def jieti():#阶梯
    s1.pulse_width(2100)     #
    time.sleep_ms(500)       #夹取的时间
#调2
def fang():
    s1.pulse_width(2100)
    time.sleep_ms(500)       #夹取的时间


def jiaozhun2():   #中心点为(80,60)
    global D2,D3,D1
    s1.pulse_width(2200)

    print("y")
    while(D2):

        if(uart.any()):
            uart.read()#?
            D2_1 = 1
            while D2_1  :
                img = sensor.snapshot()
                blobs = img.find_blobs([rr_thresholds,rr1_thresholds], roi = (30,0,78,120),pixels_threshold=600, area_threshold=800)
                if blobs:
                    for b in blobs:
                        img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                        img.draw_cross(b[5], b[6])  ##画十字交叉
                        #print(b[5]-W_X)

                        if b[5]-W_X>6:
                            uart.write('9')   #向前走
                            D2_1 = 0
                        elif W_X-b[5]>6:
                            uart.write('0')   #向后走
                            D2_1 = 0
                        else:
                            D2 = 0            #跳出前后校准
                            D2_1 = 0
                            break
    print("x")
    uart.write('2')
    while(D3):

        if (uart.any()):
            uart.read()
            print("shoudao")
            D3_1 = 1
            while D3_1 :
                img = sensor.snapshot()
                blobs = img.find_blobs([rr_thresholds,rr1_thresholds], roi = (30,0,78,120), pixels_threshold=600, area_threshold=800)
                if blobs:
                    for b in blobs:
                        img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                        img.draw_cross(b[5], b[6])  ##画十字交叉

                        #print(b[6]-W_Y)
                        if b[6]-W_Y>6:
                            uart.write('7')   #向左走
                            D3_1 = 0
                        elif W_Y-b[6]>6:
                            uart.write('8')   #向右走
                            D3_1 = 0
                        else:
                            D3 = 0
                            D1 = 0            #全部校准完成了
                            print("yxbreak")
                            D3_1 = 0
                            break





#while(True):
    #s1.pulse_width(2235)      #球拍初始化
    #time.sleep_ms(500)
    #s1.pulse_width(1700)
    #time.sleep_ms(500)


s1.pulse_width(200)      #球拍初始化



while(1):
    J0 = 1
    while (J0):
        if (uart.any()):
            B = uart.read()
            print(B)
            if B == b'1':  #红场台阶
                J0 = 0


    while(D1):
        #if (uart.any()):
          #  B = uart.read()
           # print(B)
           # if B == b'1':
        print("start xy")
        jiaozhun2()

        print("end xy")
    print("next_of_xy")
    time.sleep_ms(50)#?
    print("culaile")
    if(uart.any()):
        uart.read()
    uart.write('1')      #告诉单片机校准完成?
    print("sent 1")


    while(1):
        #print("sent 1")
        if(uart.any()):
            B = uart.read()
            print(B)
            if(B==b'2'):#？
                break

    s1.pulse_width(1700)   ####准备夹球

    while (D0):

        if (uart.any()):
            B = uart.read()
            print(B)

            print("wait 3")
            if B == b'3':    ##夹白球
                print("3")
                s1.pulse_width(2150)
                s2.pulse_width(2000)
                time.sleep_ms(500)
                uart.write('1')


            if B == b'2':  ##放白球
                print("2")
                s1.pulse_width(1700)
                time.sleep(1)
                s1.pulse_width(2250)
                hui = 2300
                uart.write('1')



while (True):
    print("see")
    start_time = utime.time()

    while (B0):  # 等待主控信息，切换红蓝方
            if (uart.any()):
                B = uart.read()
                #print(B)
                if B == b'1':            #红方，C0
                    print("1")
                    # 获取当前时间
                    start_time = utime.time()
                    B0 = 0
                    C0 = 1
                if B==b'2':              #蓝方，H0
                    print("2")
                    # 获取当前时间
                    start_time = utime.time()
                    B0=0
                    H0=1




                 ####################################   圆盘机代码到此为止



    while (D1):  # D0

        if (uart.any()):
            B = uart.read()
            print(B)
            if B == b'5':       ##？
                print("start xy")
                jiaozhun()
                print("end xy")
    print("next_of_xy")
    time.sleep_ms(50)
    print("culaile")
    if(uart.any()):
        uart.read()
    uart.write('1')
    print("sent 1")
    while(1):
        #print("sent 1")
        if(uart.any()):
            B = uart.read()
            print(B)
            if(B==b'2'):
                break

    s1.pulse_width(1700)   ####准备夹球

    while (D0):

        if (uart.any()):
            B = uart.read()
            print(B)

            print("wait 3")
            if B == b'3':    ##夹白球
                print("3")
                s1.pulse_width(2150)
                s2.pulse_width(2000)
                time.sleep_ms(500)
                uart.write('1')


            if B == b'2':  ##放白球
                print("2")
                s1.pulse_width(1700)
                time.sleep(1)
                s1.pulse_width(2250)
                hui = 2300     #？？
                uart.write('1')



            if B == b'4':        #单片机告诉你本色？
                print("4")
                D0 = 0
                E0 = 1
                uart.write('1')
            if B == b'6':
                print("6")
                D0 = 0
                R0 = 1
                uart.write('1')

    while (E0):  # E0    #立桩红球
        clock.tick()
        img = sensor.snapshot()
        #img = img1.copy(ROI)
        blobs =  img.find_blobs([r_thresholds], pixels_threshold=2000, area_threshold=3000)  # 红球输出1
        if blobs:
                #img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                #img.draw_cross(b[5], b[6])  ##画十字交叉
                ##statistics = img.get_statistics(roi=ROI)
                print('red')
                BLUE = 0  # 蓝球标志位
                RED = RED + 1  # 红球标志位
                time.sleep_ms(400)
                bense()

                if (RED==3):
                    #uart.write('1')
                    #print('you send:1')
                    RED = 0
                    BLUE = 0
                    E0 = 0     #放完了
                    F0 = 1


    while(R0):    ####立桩蓝球
        clock.tick()
        img = sensor.snapshot()
        blobs =  img.find_blobs([b_thresholds], pixels_threshold=2000, area_threshold=3000)  # 蓝球输出2
        if blobs:
            #img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
            #img.draw_cross(b[5], b[6])  ##画十字交叉
            print('blue')
            BLUE = BLUE + 1  # 蓝球标志位
            RED = 0  # 红球标志位
            time.sleep_ms(400)
            bense()

            if(BLUE==3):
                #uart.write('1')
                #print('you send:1')
                RED = 0
                BLUE = 0
                R0 = 0
                F0 = 1
#完成立桩了？


    #while (F0):  # F0
        #if (uart.any()):
            #B = uart.read()
            #print(B)
            #if B == b'4':
                #print("4")
                #F0 = 0
                #G0 = 1

    while (E0 == 1):         #red
        img1 = sensor.snapshot()
        img = img1.copy((42,0,78,120))

        blobs =  img.find_blobs([r_thresholds], pixels_threshold=600, area_threshold=800)  # 红积木输出1
        if blobs:
          for b in blobs:

           # statistics = img.get_statistics(roi=ROI)
            img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
            img.draw_cross(b[5], b[6])  ##画十字交叉
            data = "1"
            Recognition_counts = Recognition_counts+1
            if data == "1":
                data_out = json.dumps(set(data))
                uart.write('1')
                print('you send:', data_out)
                F0 = 1
                time.sleep_ms(10)

        blobs = img.find_blobs([b_thresholds], pixels_threshold=600, area_threshold=800)  # 蓝积木输出2
        if blobs:
          for b in blobs:
           # statistics = img.get_statistics(roi=ROI)
            img.draw_rectangle(b[0:4])
            img.draw_cross(b[5],b[6])
            data = "2"
            Recognition_counts = Recognition_counts+1

            if data == "2":
                data_out = json.dumps(set(data))
                uart.write('1')
                print('you send:', data_out)
                F0 = 1
                time.sleep_ms(10)

        res =  img.find_qrcodes()                                                      # 二维码小块输出3
        if len(res) > 0:  # 在图片和终端显示二维码信息
                img.draw_rectangle(res[0].rect())
                img.draw_string(2, 2, res[0].payload(), color=(0, 128, 0), scale=2)
                print(res[0].payload())
                data = "3"
                Recognition_counts = Recognition_counts+1
                if data == "1":
                    data_out = json.dumps(set(data))
                    uart.write('1')
                    print('you send:', data_out)
                    F0 = 1
                    time.sleep_ms(10)

        circles = img.find_circles(threshold=3500, x_margin=10, y_margin=10, r_margin=10,  # 红色圆环输出4
                                  r_min=2, r_max=100, r_step=2)
        if circles:
            for c in circles:
                img.draw_circle(c.x(), c.y(), c.r(),(0,0,0))
                img.draw_cross(c.x(),c.y(),(0,0,0))
                data = "4"
                Recognition_counts = Recognition_counts+1
                if data == "4":
                    data_out = json.dumps(set(data))
                    uart.write('1')
                    print('you send:', data_out)
                    F0 = 1
                    time.sleep_ms(10)

        circles = img.find_circles(roi=ROI,threshold=3500, x_margin=10, y_margin=10, r_margin=10,  # 蓝色圆环输出5
                                  r_min=2, r_max=100, r_step=2)
        if circles:
            for c in circles:
                img.draw_circle(c.x(), c.y(), c.r(),(0,0,0))
                img.draw_cross(c.x(), c.y(),color(0,0,0))
                data = "5"
                Recognition_counts = Recognition_counts+1
                if data == "5":
                    data_out = json.dumps(set(data))
                    uart.write('1')
                    print('you send:', data_out)
                    F0 = 1
                    time.sleep_ms(10)


        if Recognition_counts == 2 or Recognition_counts == 6:
            uart.write('2')  # 发送6 ，机械臂该抬起来了/降下去了
            print('2')
        elif Recognition_counts == 8:
            uart.write('3')        #发送7，告诉单片机夹完了，该走了
            print('3')
            B0 = 0
            C0 = 0
            D0 = 0
            E0 = 0
            F0 = 0
            G0 = 0
            time.sleep_ms(10)


    while (R0 == 1):         #red

        clock.tick
        img1 = sensor.snapshot()
        img = img1.copy((42,0,78,120))
        blobs =  img.find_blobs([r_thresholds], pixels_threshold=600, area_threshold=800)  # 红积木输出1
        if blobs:
          for b in blobs:
           # statistics = img.get_statistics(roi=ROI)
            img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
            img.draw_cross(b[5], b[6])  ##画十字交叉
            data = "1"
            Recognition_counts = Recognition_counts+1
            if data == "1":
                data_out = json.dumps(set(data))
                uart.write('1')
                print('you send:', data_out)
                F0 = 1
                time.sleep_ms(10)

        blobs = img.find_blobs([b_thresholds], pixels_threshold=600, area_threshold=800)  # 蓝积木输出2
        if blobs:
          for b in blobs:
           # statistics = img.get_statistics(roi=ROI)
            img.draw_rectangle(b[0:4])
            img.draw_cross(b[5],b[6])
            data = "2"
            Recognition_counts = Recognition_counts+1

            if data == "2":
                data_out = json.dumps(set(data))
                uart.write('2')
                print('you send:', data_out)
                F0 = 1
                time.sleep_ms(10)

        res =  img.find_qrcodes()                                                      # 二维码小块输出3
        if len(res) > 0:  # 在图片和终端显示二维码信息
                img.draw_rectangle(res[0].rect())
                img.draw_string(2, 2, res[0].payload(), color=(0, 128, 0), scale=2)
                print(res[0].payload())
                data = "3"
                Recognition_counts = Recognition_counts+1
                if data == "3":
                    data_out = json.dumps(set(data))
                    uart.write('3')
                    print('you send:', data_out)
                    F0 = 1
                    time.sleep_ms(10)

        #circles = img.find_circles(threshold=3500, x_margin=10, y_margin=10, r_margin=10,  # 红色圆环输出4
                                  #r_min=2, r_max=100, r_step=2)
        #if circles:
            #for c in circles:
                #img.draw_circle(c.x(), c.y(), c.r(),(0,0,0))
                #img.draw_cross(c.x(),c.y(),(0,0,0))
                #data = "4"
                #Recognition_counts = Recognition_counts+1
                #if data == "4":
                    #data_out = json.dumps(set(data))
                    #uart.write('4')
                    #print('you send:', data_out)
                    #F0 = 1
                    #time.sleep_ms(10)

        #circles = img.find_circles(roi=ROI,threshold=3500, x_margin=10, y_margin=10, r_margin=10,  # 蓝色圆环输出5
                                  #r_min=2, r_max=100, r_step=2)
        #if circles:
            #for c in circles:
                #img.draw_circle(c.x(), c.y(), c.r(),(0,0,0))
                #img.draw_cross(c.x(), c.y(),(0,0,0))
                #data = "5"
                #Recognition_counts = Recognition_counts+1
                #if data == "5":
                    #data_out = json.dumps(set(data))
                    #uart.write('5')
                    #print('you send:', data_out)
                    #F0 = 1
                    #time.sleep_ms(10)


        if Recognition_counts == 2 or Recognition_counts == 6:
            uart.write('6')  # 发送6 ，机械臂该抬起来了/降下去了
            print('6')
        elif Recognition_counts == 8:
            uart.write('7')        #发送7，告诉单片机夹完了，该走了
            print('7')
            B0 = 0
            C0 = 0
            D0 = 0
            E0 = 0
            F0 = 0
            G0 = 0
            time.sleep_ms(10)

    print(clock.fps())




# 准备一开始初始化红蓝场的时候，变换阈值







# 机器人识别色块

# 黄=（60,90,-20,0,40,70）

# 红=(15,60,20,50,0,40)

# 蓝=（15，30，-10，20，-45，-10 ）
# 35,65,-55,-38,20,40lu

import sensor, image, time, math
import json,utime
from pyb import UART
from pyb import Pin, Servo

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=1000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)     #寻找色块不能开白平衡


#################阈值##################

#######################################


y_thresholds  = (60, 94, -18, 7, 9, 76)  # 黄
y2_thresholds = (38, 93, 60, 7, 106, 40) # 黄亮阈值
y3_thresholds = (50, 99, 3, -31, 99, 53)

w_thresholds = (48, 100, -11, 17, -35, 0)
w1_thresholds = (92, 100, -17, 5, -7, 5)
w2_thresholds = (44, 90, -14, 14, -40, -6)
w3_thresholds = (90, 100, -37, 58, -46, 32)

r_thresholds = ((38, 64, 82, 15, 7, 43)) # 红

b_thresholds = (40, 72, -26, 21, -63, -15)  # 蓝
b2_thresholds = ((7, 67, 46, 5, -81, -16))




###################阶梯阈值###############
#########################################


rr_thresholds =(38, 46, 36, 72, 22, 66)
rr1_thresholds = (20, 46, 21, 63, 0, 45)

bb_thresholds = (40, 72, -26, 21, -63, -15)


#######################################
#######################################

clock = time.clock()
ROI = (3, 87, 219, 75)

uart = UART(3, 9600)  # P4为TX P5为RX


#######################################
#######################################


t_t = 400#320
pia = 1600
hui = 2050
t_1 = 20
t_2 = 200
ROI1 = (40,40,80,80)
ROI2 = (45,40,80,80)
W_X = 80+1
W_Y = 60-7

#######################################
#######################################


########不知道干嘛的#########

i = 0



B0 = 0  # 为接收串口标志
C0 = 0  #红色方标志位
H0 = 0  #蓝色方标志位


########jiaozhun的内部标志位
D2 = 1
D3 = 1
D1= 1



D0 = 1  # 0
E0 = 0

C1 = 0
C2 = 0
C3 = 0

BLUE = 0  # 蓝球标志位（用于计数）
RED = 0  # 红球标志位



W0 = 0
F0 = 0
G0 = 0
R0 = 0
J0 = 1
J1 = 0   #开始识别
J2 = 0   #方体
J3 = 0   #环 或 QR

JT = 0
JJ1 = 0  #夹球连招
JJ2 = 0  #下一个点位
JJ3 = 0  #好环，记住它

s1=Servo(1)
s2=Servo(2)
def bense():#打到右边,本色
    #s1.pulse_width(2200)
    s1.pulse_width(pia)

    time.sleep_ms(t_1)

    s2.pulse_width(2000)

    time.sleep_ms(t_2)
    s1.pulse_width(hui)
    time.sleep_ms(t_t-t_1-t_2)

def huangse():#打到左边，黄色
    #s1.pulse_width(850)
    s1.pulse_width(pia)

    time.sleep_ms(t_1)
    s2.pulse_width(1120)
    time.sleep_ms(t_2)
    s1.pulse_width(hui)
    time.sleep_ms(t_t-t_1-t_2)

def jiaozhun():   #中心点为(80,60)
    global D2,D3,D1
    s1.pulse_width(2200)

    print("y")
    while(D2):

        if(uart.any()):
            uart.read()
            D2_1 = 1
            while D2_1  :
                img = sensor.snapshot()
                blobs = img.find_blobs([w3_thresholds], pixels_threshold=600, area_threshold=800)
                if blobs:
                    for b in blobs:
                        img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                        img.draw_cross(b[5], b[6])  ##画十字交叉
                        #print(b[5]-W_X)

                        if b[5]-W_X>6:
                            uart.write('9')   #向前走
                            D2_1 = 0
                        elif W_X-b[5]>6:
                            uart.write('0')   #向后走
                            D2_1 = 0
                        else:
                            D2 = 0
                            D2_1 = 0
                            break
    print("x")
    uart.write('2')
    while(D3):

        if (uart.any()):
            uart.read()
            print("shoudao")
            D3_1 = 1
            while D3_1 :
                img = sensor.snapshot()
                blobs = img.find_blobs([w3_thresholds], pixels_threshold=600, area_threshold=800)
                if blobs:
                    for b in blobs:
                        img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                        img.draw_cross(b[5], b[6])  ##画十字交叉

                        #print(b[6]-W_Y)
                        if b[6]-W_Y>6:
                            uart.write('7')   #向左走
                            D3_1 = 0
                        elif W_Y-b[6]>6:
                            uart.write('8')   #向右走
                            D3_1 = 0
                        else:
                            D3 = 0
                            D1 = 0
                            print("yxbreak")
                            D3_1 = 0
                            break




def jiaozhun2():   #中心点为(80,60)
    global D2,D3,D1
    s1.pulse_width(2200)

    print("y")
    while(D2):

        if(uart.any()):
            uart.read()
            D2_1 = 1
            while D2_1  :
                img = sensor.snapshot()
                blobs = img.find_blobs([rr_thresholds,rr1_thresholds], roi = (30,0,78,120),pixels_threshold=600, area_threshold=800)
                if blobs:
                    for b in blobs:
                        img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                        img.draw_cross(b[5], b[6])  ##画十字交叉
                        #print(b[5]-W_X)

                        if b[5]-W_X>6:
                            uart.write('9')   #向前走
                            D2_1 = 0
                        elif W_X-b[5]>6:
                            uart.write('0')   #向后走
                            D2_1 = 0
                        else:
                            D2 = 0
                            D2_1 = 0
                            break
    print("x")
    uart.write('2')
    while(D3):

        if (uart.any()):
            uart.read()
            print("shoudao")
            D3_1 = 1
            while D3_1 :
                img = sensor.snapshot()
                blobs = img.find_blobs([rr_thresholds,rr1_thresholds], roi = (30,0,78,120), pixels_threshold=600, area_threshold=800)
                if blobs:
                    for b in blobs:
                        img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                        img.draw_cross(b[5], b[6])  ##画十字交叉

                        #print(b[6]-W_Y)
                        if b[6]-W_Y>6:
                            uart.write('7')   #向左走
                            D3_1 = 0
                        elif W_Y-b[6]>6:
                            uart.write('8')   #向右走
                            D3_1 = 0
                        else:
                            D3 = 0
                            D1 = 0
                            print("yxbreak")
                            D3_1 = 0
                            break








s1.pulse_width(2100)      #球拍初始化
#阶梯
while(JT):
    J0 = 1

    while (J0):     ##等待识别指令
        if (uart.any()):
            B = uart.read()
        if(B):
            print(B)
            if B == b'1':
                J0 = 0
                J1 = 1

    while (J1):
        img = sensor.snapshot()

        ########计时
        clock.tick()
        time.sleep_ms(10)

        # 获取当前时间
        end_time = utime.time()
        # 计算时间差
        elapsed_time = end_time- start_time   #start_time在哪？
        print(elapsed_time)


        if elapsed_time > 3:
            J1 = 0
            J2 = 1      ###################方体
            i = 0
            break

        # 寻找圆环
        circles = img.find_circles(roi = (42,0,77,120),threshold=1500, x_margin=10, y_margin=10, r_margin=10, r_min=8, r_max=15)

        if circles:
            i = i + 1
            if (i==3):
                J1 = 0
                J3 = 1   ##################环或QR
                i = 0
                break
            for c in circles:
                # 绘制圆环
                img.draw_circle(c.x(), c.y(), c.r(), color=(0, 0, 255))
                img.draw_string(c.x(), c.y() - 10, "Circle", color=(0, 0, 255))
    while(J2):
        img = sensor.snapshot()
        clock.tick()
        time.sleep_ms(10)

        # 获取当前时间
        end_time = utime.time()
        # 计算时间差
        elapsed_time = end_time- start_time
        print(elapsed_time)


        if elapsed_time > 3:
            J2 = 0
            JJ2 = 1      ###################下一个点位
            i = 0
            break

        blobs = img.find_blobs([rr_thresholds,rr1_thresholds], roi = (30,0,78,120),pixels_threshold=600, area_threshold=800)
        if blobs:
            i = i + 1
            if (i == 3):
                JJ1 = 1
                J2 = 0
                i = 0
                break

            for b in blobs:
                img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                img.draw_cross(b[5], b[6])  ##画十字交叉

    while(J3):
        img = sensor.snapshot()
        clock.tick()
        time.sleep_ms(10)

        # 获取当前时间
        end_time = utime.time()
        # 计算时间差
        elapsed_time = end_time- start_time
        print(elapsed_time)


        if elapsed_time > 3:
            J3 = 0
            JJ2 = 1      ###################下一个点位
            i = 0
            break

        blobs = img.find_blobs([rr_thresholds,rr1_thresholds], roi = (30,0,78,120),pixels_threshold=600, area_threshold=800)
        if blobs:
            i = i + 1
            if (i == 3):
                JJ1 = 1
                J3 = 0
                i = 0
                break

            for b in blobs:
                img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                img.draw_cross(b[5], b[6])  ##画十字交叉

    while(JJ1):             #准备较准
        uart.write('3')
        JJ1 = 0
        D1 = 1
    while(JJ2):             ##下一个点位
        uart.write('4')
        JJ2 = 0
        D1 = 0










###########较准###########
    while(D1):
        #if (uart.any()):
          #  B = uart.read()
           # print(B)
           # if B == b'1':
        print("start xy")
        jiaozhun2()

        print("end xy")
    print("next_of_xy")
    time.sleep_ms(50)
    print("culaile")
    if(uart.any()):
        uart.read()
    uart.write('1')
    print("sent 1")


    while(1):
        #print("sent 1")
        if(uart.any()):
            B = uart.read()
            print(B)
            if(B==b'2'):
                break

    s1.pulse_width(1700)   ####准备夹球

    while (D0):

        if (uart.any()):
            B = uart.read()
            print(B)

            print("wait 3")
            if B == b'3':    ##夹白球
                print("3")
                s1.pulse_width(2150)
                s2.pulse_width(2000)
                time.sleep_ms(500)
                uart.write('1')


            if B == b'2':  ##放白球
                print("2")
                s1.pulse_width(1700)
                time.sleep(1)
                s1.pulse_width(2250)
                hui = 2300
                uart.write('1')
                D0 = 0



while (True):
    print("see")
    start_time = utime.time()

    while (B0):  # 等待主控信息，切换红蓝方
            if (uart.any()):
                B = uart.read()
                #print(B)
                if B == b'1':            #红方，C0
                    print("1")
                    # 获取当前时间
                    start_time = utime.time()
                    B0 = 0
                    C0 = 1
                if B==b'2':              #蓝方，H0
                    print("2")
                    # 获取当前时间
                    start_time = utime.time()
                    B0=0
                    H0=1



    while (C0):  # C0，红方圆盘机
        clock.tick()
        time.sleep_ms(100)

        # 获取当前时间
        end_time = utime.time()
        # 计算时间差
        elapsed_time = end_time- start_time
        print(elapsed_time)

        img1 = sensor.snapshot()
        img = img1.copy((40,40,80,80))
        blobs = img.find_blobs([y_thresholds], pixels_threshold=600, area_threshold=800)
        if (blobs):
              #for b in blobs:
                #img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                #img.draw_cross(b[5], b[6])  ##画十字交叉
                #statistics = img.get_statistics(roi=ROI)
                time.sleep_ms(330)
                huangse()
                print('yellow')


        if elapsed_time > 10:      #（15s后识别红球）
            time.sleep_ms(100)
            while(C0):
                 clock.tick()
                 img1 = sensor.snapshot()
                 img = img1.copy(ROI2)
                 blobs = img.find_blobs([r_thresholds], pixels_threshold=600, area_threshold=800)
                 if blobs:
                      #for b in blobs:
                         #img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                         #img.draw_cross(b[5], b[6])  ##画十字交叉
                         #statistics = img.get_statistics(roi=ROI)##不行则回167行CV
                         print('red')
                         time.sleep_ms(300)
                         bense()
                         time.sleep_ms(1300)

                         RED = RED +1
                         time.sleep_ms(10)


                         if (RED + BLUE == 6):             #满6个本色，跳出
                            uart.write('3')
                            print('you send:3')
                            RED = 0
                            BLUE = 0
                            C0 = 0
                            D0 = 1







    while (H0):
        clock.tick()
        time.sleep_ms(100)

        # 获取当前时间
        end_time = utime.time()
        # 计算时间差
        elapsed_time = end_time- start_time
        print(elapsed_time)

        img1 = sensor.snapshot()
        img = img1.copy((40,40,80,80))
        blobs = img.find_blobs([y_thresholds], pixels_threshold=600, area_threshold=800)
        if (blobs):
              #for b in blobs:
                #img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                #img.draw_cross(b[5], b[6])  ##画十字交叉
                #statistics = img.get_statistics(roi=ROI)
                time.sleep_ms(330)
                huangse()
                print('yellow')


        if elapsed_time > 10:      #（15s后识别蓝球）
            time.sleep_ms(100)
            while(C0):
                 clock.tick()
                 img1 = sensor.snapshot()
                 img = img1.copy(ROI2)
                 blobs = img.find_blobs([b_thresholds], pixels_threshold=600, area_threshold=800)
                 if blobs:
                      #for b in blobs:
                         #img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                         #img.draw_cross(b[5], b[6])  ##画十字交叉
                         #statistics = img.get_statistics(roi=ROI)##不行则回167行CV
                         print('blue')
                         time.sleep_ms(300)
                         bense()

                         BLUE = BLUE +1
                         time.sleep_ms(10)


                         if (RED + BLUE == 6):             #满6个本色，跳出
                            uart.write('3')
                            print('you send:3')
                            RED = 0
                            BLUE = 0
                            H0 = 0
                            D0 = 1



                 ####################################   圆盘机代码到此为止



    while (D1):  # D0

        if (uart.any()):
            B = uart.read()
            print(B)
            if B == b'5':
                print("start xy")
                jiaozhun()
                print("end xy")
    print("next_of_xy")
    time.sleep_ms(50)
    print("culaile")
    if(uart.any()):
        uart.read()
    uart.write('1')
    print("sent 1")
    while(1):
        #print("sent 1")
        if(uart.any()):
            B = uart.read()
            print(B)
            if(B==b'2'):
                break

    s1.pulse_width(1700)   ####准备夹球

    while (D0):

        if (uart.any()):
            B = uart.read()
            print(B)

            print("wait 3")
            if B == b'3':    ##夹白球
                print("3")
                s1.pulse_width(2150)
                s2.pulse_width(2000)
                time.sleep_ms(500)
                uart.write('1')


            if B == b'2':  ##放白球
                print("2")
                s1.pulse_width(1700)
                time.sleep(1)
                s1.pulse_width(2250)
                hui = 2300
                uart.write('1')



            if B == b'4':
                print("4")
                D0 = 0
                E0 = 1
                uart.write('1')
            if B == b'6':
                print("6")
                D0 = 0
                R0 = 1
                uart.write('1')



        ######################################################################################################



    while (E0):  # E0    #立桩红球
        clock.tick()
        img = sensor.snapshot()
        #img = img1.copy(ROI)
        blobs =  img.find_blobs([r_thresholds], pixels_threshold=2000, area_threshold=3000)  # 红球输出1
        if blobs:
                #img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
                #img.draw_cross(b[5], b[6])  ##画十字交叉
                ##statistics = img.get_statistics(roi=ROI)
                print('red')
                BLUE = 0  # 蓝球标志位
                RED = RED + 1  # 红球标志位
                time.sleep_ms(400)
                bense()

                if (RED==3):
                    #uart.write('1')
                    #print('you send:1')
                    RED = 0
                    BLUE = 0
                    E0 = 0
                    F0 = 1


    while(R0):    ####立桩蓝球
        clock.tick()
        img = sensor.snapshot()
        blobs =  img.find_blobs([b_thresholds], pixels_threshold=2000, area_threshold=3000)  # 蓝球输出2
        if blobs:
            #img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
            #img.draw_cross(b[5], b[6])  ##画十字交叉
            print('blue')
            BLUE = BLUE + 1  # 蓝球标志位
            RED = 0  # 红球标志位
            time.sleep_ms(400)
            bense()

            if(BLUE==3):
                #uart.write('1')
                #print('you send:1')
                RED = 0
                BLUE = 0
                R0 = 0
                F0 = 1

########################################################################################

    #while (F0):  # F0
        #if (uart.any()):
            #B = uart.read()
            #print(B)
            #if B == b'4':
                #print("4")
                #F0 = 0
                #G0 = 1

    while (E0 == 1):         #red
        img1 = sensor.snapshot()
        img = img1.copy((42,0,78,120))

        blobs =  img.find_blobs([rr_thresholds,rr1_thresholds], pixels_threshold=600, area_threshold=800)  # 红积木输出1
        if blobs:
          for b in blobs:


            img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
            img.draw_cross(b[5], b[6])  ##画十字交叉
            color = 1   ###识别颜色  1 红色

            if color == 1:

                uart.write('1')
                print('1')
                F0 = 1
                time.sleep_ms(10)

        blobs = img.find_blobs([b_thresholds], pixels_threshold=600, area_threshold=800)  # 蓝积木输出2
        if blobs:
          for b in blobs:
            img.draw_rectangle(b[0:4])
            img.draw_cross(b[5],b[6])
            color = 2       ###4 为蓝色
            if color == 2:

                uart.write('2')
                print('you send:', data_out)
                F0 = 1
                time.sleep_ms(10)

        res =  img.find_qrcodes()                                                      # 二维码小块输出3
        if len(res) > 0:  # 在图片和终端显示二维码信息
                img.draw_rectangle(res[0].rect())
                img.draw_string(2, 2, res[0].payload(), color=(0, 128, 0), scale=2)
                print(res[0].payload())
                data = "3"
                Recognition_counts = Recognition_counts+1
                if data == "1":
                    data_out = json.dumps(set(data))
                    uart.write('1')
                    print('you send:', data_out)
                    F0 = 1
                    time.sleep_ms(10)

        circles = img.find_circles(threshold=3500, x_margin=10, y_margin=10, r_margin=10,  # 红色圆环输出4
                                  r_min=2, r_max=100, r_step=2)
        if circles:
            for c in circles:
                img.draw_circle(c.x(), c.y(), c.r(),(0,0,0))
                img.draw_cross(c.x(),c.y(),(0,0,0))
                data = "4"
                Recognition_counts = Recognition_counts+1
                if data == "4":
                    data_out = json.dumps(set(data))
                    uart.write('1')
                    print('you send:', data_out)
                    F0 = 1
                    time.sleep_ms(10)

        circles = img.find_circles(roi=ROI,threshold=3500, x_margin=10, y_margin=10, r_margin=10,  # 蓝色圆环输出5
                                  r_min=2, r_max=100, r_step=2)
        if circles:
            for c in circles:
                img.draw_circle(c.x(), c.y(), c.r(),(0,0,0))
                img.draw_cross(c.x(), c.y(),color(0,0,0))
                data = "5"
                Recognition_counts = Recognition_counts+1
                if data == "5":
                    data_out = json.dumps(set(data))
                    uart.write('1')
                    print('you send:', data_out)
                    F0 = 1
                    time.sleep_ms(10)


        if Recognition_counts == 2 or Recognition_counts == 6:
            uart.write('2')  # 发送6 ，机械臂该抬起来了/降下去了
            print('2')
        elif Recognition_counts == 8:
            uart.write('3')        #发送7，告诉单片机夹完了，该走了
            print('3')
            B0 = 0
            C0 = 0
            D0 = 0
            E0 = 0
            F0 = 0
            G0 = 0
            time.sleep_ms(10)


    while (R0 == 1):         #red

        clock.tick
        img1 = sensor.snapshot()
        img = img1.copy((42,0,78,120))
        blobs =  img.find_blobs([r_thresholds], pixels_threshold=600, area_threshold=800)  # 红积木输出1
        if blobs:
          for b in blobs:
           # statistics = img.get_statistics(roi=ROI)
            img.draw_rectangle(b[0:4])  ##在图像上绘制一个矩形。
            img.draw_cross(b[5], b[6])  ##画十字交叉
            data = "1"
            Recognition_counts = Recognition_counts+1
            if data == "1":
                data_out = json.dumps(set(data))
                uart.write('1')
                print('you send:', data_out)
                F0 = 1
                time.sleep_ms(10)

        blobs = img.find_blobs([b_thresholds], pixels_threshold=600, area_threshold=800)  # 蓝积木输出2
        if blobs:
          for b in blobs:
           # statistics = img.get_statistics(roi=ROI)
            img.draw_rectangle(b[0:4])
            img.draw_cross(b[5],b[6])
            data = "2"
            Recognition_counts = Recognition_counts+1

            if data == "2":
                data_out = json.dumps(set(data))
                uart.write('2')
                print('you send:', data_out)
                F0 = 1
                time.sleep_ms(10)

        res =  img.find_qrcodes()                                                      # 二维码小块输出3
        if len(res) > 0:  # 在图片和终端显示二维码信息
                img.draw_rectangle(res[0].rect())
                img.draw_string(2, 2, res[0].payload(), color=(0, 128, 0), scale=2)
                print(res[0].payload())
                data = "3"
                Recognition_counts = Recognition_counts+1
                if data == "3":
                    data_out = json.dumps(set(data))
                    uart.write('3')
                    print('you send:', data_out)
                    F0 = 1
                    time.sleep_ms(10)

        #circles = img.find_circles(threshold=3500, x_margin=10, y_margin=10, r_margin=10,  # 红色圆环输出4
                                  #r_min=2, r_max=100, r_step=2)
        #if circles:
            #for c in circles:
                #img.draw_circle(c.x(), c.y(), c.r(),(0,0,0))
                #img.draw_cross(c.x(),c.y(),(0,0,0))
                #data = "4"
                #Recognition_counts = Recognition_counts+1
                #if data == "4":
                    #data_out = json.dumps(set(data))
                    #uart.write('4')
                    #print('you send:', data_out)
                    #F0 = 1
                    #time.sleep_ms(10)

        #circles = img.find_circles(roi=ROI,threshold=3500, x_margin=10, y_margin=10, r_margin=10,  # 蓝色圆环输出5
                                  #r_min=2, r_max=100, r_step=2)
        #if circles:
            #for c in circles:
                #img.draw_circle(c.x(), c.y(), c.r(),(0,0,0))
                #img.draw_cross(c.x(), c.y(),(0,0,0))
                #data = "5"
                #Recognition_counts = Recognition_counts+1
                #if data == "5":
                    #data_out = json.dumps(set(data))
                    #uart.write('5')
                    #print('you send:', data_out)
                    #F0 = 1
                    #time.sleep_ms(10)


        if Recognition_counts == 2 or Recognition_counts == 6:
            uart.write('6')  # 发送6 ，机械臂该抬起来了/降下去了
            print('6')
        elif Recognition_counts == 8:
            uart.write('7')        #发送7，告诉单片机夹完了，该走了
            print('7')
            B0 = 0
            C0 = 0
            D0 = 0
            E0 = 0
            F0 = 0
            G0 = 0
            time.sleep_ms(10)

    print(clock.fps())




