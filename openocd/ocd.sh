sudo openocd -f rpi4.cfg -c "transport select swd" -c "adapter speed 1000"  -f atsame5x.cfg 
