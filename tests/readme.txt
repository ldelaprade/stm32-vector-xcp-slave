
Route issues with device with static IP address.



Network tips for windows
(I set my dedicated ethernet as 192.168.95.10):

route DELETE 192.168.95.2
route ADD 192.168.95.2 MASK 255.255.255.255 192.168.95.10 METRIC 1
route CHANGE -p 192.168.95.2 MASK 255.255.255.255 192.168.95.10 METRIC 1


Network tips for Linux
(I set my dedicated ethernet device 'enx3c18a0d4f474' as 192.168.95.15):

sudo ip a add 192.168.95.15/24 dev enx3c18a0d4f474
