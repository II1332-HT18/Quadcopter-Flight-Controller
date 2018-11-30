# Quadcopter-Flight-Controller
## README / INSTRUCTIONS
## Analys
### Timing and time analys
```
Analysthread:   8ms   execution time    runs 1 time  in 20ms hyperperiod  
Controlthread:  4ms   execution time    runs 2 times in 20ms hyperperiod  
Filterthread:   160us execution time    runs 5 times in 20ms hyperperiod  
Pwminthread:    1ms   execution time    runs 2 times in 20ms hyperperiod  
```
The analysis thread is delayed with a 10 second vTaskDelay(10000); in order to have time to pair with bluetooth module.
Pair with the bluetooth module.
To connect with bluetooth open SerialPlot and select the right COM-port and click open.(a blue light indicates success)

If you use UART-cable for analysis then just select the right COM-port, click open and wait.

#### SerialPlot-config
1. Open up SerialPlott
2. Click 'File' -> 'Load Settings' and choose the file 'Quadplotter-beachbois-edition.ini'

## Automatic Control
*readme*

## Filter
*readme*

## Modelling
*readme*
