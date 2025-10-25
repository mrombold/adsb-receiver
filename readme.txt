build instructions:

go build -o stratux-aggregator ./cmd/stratux-aggregator




adsb1090 basestation data format:

http://woodair.net/sbs/article/barebones42_socket_data.htm


Check data sent to EFB:
full tcp packet
sudo tcpdump -i any -s 0 -X 'port 4000'  (hex and ascii)
sudo tcpdump -i any -s 0 -A 'port 4000'  (ascii)

stripped packet without tcp headers
sudo tshark -i wlan0 -f "udp port 4000" -Y "udp" -T fields -e udp.length -e udp.payload


Check network bandwidth usage:
ifstat -i wlan0 1

Spy on dump978 output:

sudo netcat localhost 30978 | hexdump -C | head
nc localhost 30979 | head
nc localhost 30979 | jq .


dump978 record:
(dump978 localhost is over TCP, not UDP)
Couldn't get 30979 to work for more than a minute at first.

creates raw output:
nc localhost -u 30978 > dump978.raw

tcpdump for playback later:
sudo tcpdump -i lo -w dump978.pcap port 30978


Look at log files of running daemons:
on startup:   journalctl -u service-name
follow real time:  journalctl -u service-name -f
recent: journalctl -u service-name -n 500
time:  journalctl -u service-name --since "10 minutes ago"


AHRS/I2C:

sudo i2cdetect -y 1

source ~/adsb-receiver/venv/bin/activate

go run simple_sensor_read.go

