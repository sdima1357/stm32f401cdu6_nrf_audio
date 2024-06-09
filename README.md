# stm32f401cdu6_nrf_audio
Low latency wireless lossless audio, 2 chanels 44100 16bit , pure passive receiver with error correction. PC(or phone or tablet)->USB audio -> stm32->nrf24L01 ----> nrf24L01->stm32->i2s->pcm5102
Planned latency is about ~10 mS (it will be checked)


Measured - 13 milliseconds!
![image1](https://github.com/sdima1357/stm32f401cdu6_nrf_audio/blob/main/images/IMG_20240609_221716_2c.jpg?raw=true)

