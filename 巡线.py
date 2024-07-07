THRESHOLD = (5, 70, -23, 15, -57, 0) # 追踪的线的颜色阈值

import sensor, image, time
from pyb import LED
import car
from pid import PID

# 一条线的表示方法 " y = ax + b "   其中：
rho_pid = PID(p=0.4, i=0)   # 相当于“ b ”——> 截距        控制直线在视野中的位置（左右距离的偏移）最终目的是线在视野的中央
theta_pid = PID(p=0.001, i=0)   # 相当于“ a ”——> 斜率    控制直线偏移的角度    
        # 如果在运动过程中小车偏移得比较大或者是转弯转得比较大，那么就减小相应pid的值


# 打开OpenMV的RGB灯——>用于补光  因为颜色会受到环境光照的影响，所以我们建议最好打开OpenMV自带的补光灯来保持环境的稳定
LED(1).on()
LED(2).on()
LED(3).on()
    #如果开灯会造成严重反光，则关闭灯！

sensor.reset()

# 在安装时我们的OpenMV是倒着安装的，因此我们需要相应地在代码里把图像正过来！以下两句函数均是对图像进行镜像处理
sensor.set_vflip(True)      # 设置OpenMV图像“水平方向进行翻转”
sensor.set_hmirror(True)    # 设置OpenMV图像“竖直方向进行翻转”
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA) # 线性回归算法的运算量大，越小的分辨率识别的效果越好，运算速度越快
#sensor.set_windowing([0,20,80,40])
sensor.skip_frames(time = 2000)     # 等待2秒    
        # 警告：如果使用QQVGA，有时处理帧可能需要几秒钟。
clock = time.clock()                

while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD]) # 截取一张图片，进行 “阈值分割”
                        # 阈值分割函数image.binary()对图像进行二值化（binary:二元的;由两部分组成的）
                        # 得到的效果是：将阈值颜色变成白色，非阈值颜色变成黑色
    
    # 调用线性回归函数
    line = img.get_regression([(100,100)], robust = True)# 对所有的阈值像素进行线性回归
                                                         # 线性回归的效果就是将我们视野中“二值化”分割后的图像回归成一条直线
                # get_regression()返回的是一条直线line
   
    #如果发现了线                                       
    if (line):# 利用得到的line对象来计算pid的值——>进而控制小车运动
        rho_err = abs(line.rho())-img.width()/2 # 计算我们的直线相对于中央位置偏移的距离（偏移的像素）
                    # abs()函数：返回数字的绝对值  line.rho()：返回霍夫变换后的直线p值。
        
        # 进行坐标的变换：y轴方向为0°，x轴正方向为90°，x轴负方向为-90°
        if line.theta()>90:# line.theta()是我们得到的直线的角度
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
            
            
        # 将我们得到的直线画出来    
        img.draw_line(line.line(), color = 127)# line.line()：返回一个直线元组(x1, y1, x2, y2)  
        
        print(rho_err,line.magnitude(),rho_err) #line.magnitude()返回一个表示“线性回归效果”的值，这个值越大，线性回归效果越好；
                                                               # 如果越接近于0，说明我们的线性回归效果越接近于一个圆，效果越差 
        if line.magnitude()>8:  # 如果线性回归的效果比较好
            #进行pid的运算
            rho_output = rho_pid.get_pid(rho_err,1) # 将刚刚计算出的rho_err传递进rho_pid中
            theta_output = theta_pid.get_pid(theta_err,1)
            output = rho_output+theta_output    # 将得到的两个pid参数进行相加，利用得到的参数output来控制小车电机的运动
            
            car.run(50+output, 50-output)# 对于左轮子来说就是+output，对于右轮子来说就是-output
                                         # 以中央速度50为基准值进行加减（car.run()函数中小车电机的速度为0-100）
                                         # 如果想让小车快一点就设置为大于50的数，慢一点就设置为小于50的数   
        else:# 如果线性回归的效果并不好
            car.run(0,0)    # 那么就停止运动
   
    # 如果没有发现线
    else:
        car.run(50,-50)# 原地旋转，寻找视野中的线
        pass
    #print(clock.fps())
