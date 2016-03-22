serial-speed-meter
==

Simple serial read bytes counter.

Usage:

    $ python ./start.py serial_port [sampling_duration_sec]

Example:

    $ python ./start.py /dev/tty.usbmodem1234 10
    Enter Ctrl+C twice to exit!
    1458614648 : 838 bytes/sec
    1458614649 : 729 bytes/sec
    1458614650 : 729 bytes/sec
    1458614651 : 804 bytes/sec
    1458614652 : 735 bytes/sec
    1458614653 : 729 bytes/sec
    1458614654 : 729 bytes/sec
    1458614655 : 810 bytes/sec
    1458614656 : 729 bytes/sec
    1458614657 : 729 bytes/sec
    ========================================
    Total Time in Seconds => 10
    Total Received Bytes => 7561
    Mean Received Bytes Per Second => 756.10
    Baud Rate in Bit Per Second => 115200
    ========================================
