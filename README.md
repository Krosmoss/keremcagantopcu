# Summary of the code.
In this part of the project i tried to use CDC_Receive but it didn't worked as i planned. I was expecting that the usb will transmit the data's separately, with each string the data will transmit another value that i sent that will ultimately change the b,w1,w2 values in my main.
But it didn't worked. I commented them from line 154-157 because the pwm values were not changing. Thus i just end up with a AND perceptron which trains
itself, using the pwm values that are read from ADC.
