這裏實現了ros2和stm32的通訊，stm32開發板這邊的程式是不斷地從uart口發訊息出來,無論是什麼型號的板
理論上都是可以接收到的,ros2這邊讀取從usb口讀取訊息,目前的話並沒有打包成state_interface或者cmd_interface

這裏用到的hardware interface在my_hardware_interface裏面,
運行之前在你的工作空間輸入指令colcon build --symlink-install,
來建立項目並輸入source install/setup.bash

運行之前確保你的stm32開發板用uart與主機相連,
並運行 ls /dev/ttyUSB*來檢查串口的位置,
如果結果是/dev/ttyUSB0就没有問題,
有時有可能是/dev/ttyUSB1之類的,就要想方法改成/dev/ttyUSB0

確保你的stm32開發版輸出的波特率為9600,都没問題的話就輸入ros2 launch controlko_bringup rrbot_real.launch.py \ uart_port:=/dev/ttyUSB0 uart_baudrate:=9600

