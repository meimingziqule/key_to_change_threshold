import sensor, image, time, pyb
from pyb import Pin
#初始化一次auto_roi->找到auto_roi中的红色色块并返回外接矩形框坐标->将该坐标赋值给auto_roi->由statistics得到新的颜色阈值->从第二步开始循环
red_thresholds = (47, 0, 78, 11, 65, -31)    #定义变量
lab =  [0,0,0]
ROI = (140,100,30,30)
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QQVGA2 (128×160)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

while(True):
    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.
    p_in = Pin('P0', Pin.IN, Pin.PULL_UP)#设置p_in为输入引脚，并开启上拉电阻
    value = p_in.value() # get value, 0 or 1#读入p_in引脚的值
    get_auto_roi_blobs = img.find_blobs([red_thresholds],roi=ROI,pixels_threshold=50, area_threshold=50,merge = True)
    if value == 0:
        img.draw_rectangle(ROI, color = (255,255,255))   #画roi框
        
        statistics_Data = img.get_statistics(roi = ROI )
    
        lab[0] =  statistics_Data.l_median()
        lab[1] =  statistics_Data.a_median()
        lab[2] =  statistics_Data.b_median()

        
        #计算颜色阈值，这样写的话，颜色阈值是实时变化的，后续想要什么效果可以自己修改
        red_thresholds = (lab[0]-20,lab[0]+20, lab[1]-20, \
                           lab[1]+20,lab[2]-20, lab[2]+20,)
    
    img.binary([red_thresholds]) #二值化看图像效果
    print('red_thresholds:')
    print(red_thresholds)        #打印输出颜色阈值
    print("fps:")
    fps = 'fps:'+str(clock.fps())
    img.draw_string(0, 0, fps, lab=(255, 0, 0), scale=2)
    print(clock.fps())
    img.draw_rectangle(ROI, lab = (255,255,255))   #画roi框


#一,通过四个按键控制（x,y,w,h）
#二，通过按键更换成更改与之模型，前提已经画出想要的框

#待测试：激光笔

#总结  ROI得大到一定程度  否则识别效果很差