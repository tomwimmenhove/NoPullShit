EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "NoPullShit"
Date "2021-06-01"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Microchip_ATmega:ATmega328P-AU U3
U 1 1 60B552BE
P 6200 3800
F 0 "U3" H 6600 2350 50  0000 C CNN
F 1 "ATmega328P-AU" H 6100 3800 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 6200 3800 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 6200 3800 50  0001 C CNN
	1    6200 3800
	1    0    0    -1  
$EndComp
$Comp
L custom:HX711 U1
U 1 1 60B59282
P 3150 4600
F 0 "U1" H 3650 3950 50  0000 L CNN
F 1 "HX711" H 3100 4600 50  0000 L CNN
F 2 "" H 2450 6300 50  0001 C CNN
F 3 "" H 2450 6300 50  0001 C CNN
	1    3150 4600
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:S8550 Q1
U 1 1 60B5B068
P 3200 3650
F 0 "Q1" V 3528 3650 50  0000 C CNN
F 1 "S8550" V 3437 3650 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 3400 3575 50  0001 L CIN
F 3 "http://www.unisonic.com.tw/datasheet/S8550.pdf" H 3200 3650 50  0001 L CNN
	1    3200 3650
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_BJT:S8550 Q3
U 1 1 60B5C0C5
P 9650 3100
F 0 "Q3" H 9840 3054 50  0000 L CNN
F 1 "S8550" H 9850 3150 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 9850 3025 50  0001 L CIN
F 3 "http://www.unisonic.com.tw/datasheet/S8550.pdf" H 9650 3100 50  0001 L CNN
	1    9650 3100
	1    0    0    1   
$EndComp
$Comp
L Transistor_BJT:S8050 Q2
U 1 1 60B5C5DE
P 9650 2700
F 0 "Q2" H 9840 2746 50  0000 L CNN
F 1 "S8050" H 9840 2655 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 9850 2625 50  0001 L CIN
F 3 "http://www.unisonic.com.tw/datasheet/S8050.pdf" H 9650 2700 50  0001 L CNN
	1    9650 2700
	1    0    0    -1  
$EndComp
$Comp
L custom:TP4056 U2
U 1 1 60B647A5
P 3550 1200
F 0 "U2" H 3550 1665 50  0000 C CNN
F 1 "TP4056" H 3550 1574 50  0000 C CNN
F 2 "" H 2750 1450 50  0001 C CNN
F 3 "" H 2750 1450 50  0001 C CNN
	1    3550 1200
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J1
U 1 1 60B6537A
P 800 1200
F 0 "J1" H 857 1667 50  0000 C CNN
F 1 "USB Power" H 857 1576 50  0000 C CNN
F 2 "" H 950 1150 50  0001 C CNN
F 3 "~" H 950 1150 50  0001 C CNN
	1    800  1200
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Dual_2pin D1
U 1 1 60B6858B
P 7700 1100
F 0 "D1" H 7700 1496 50  0000 C CNN
F 1 "Indicator" H 7700 1405 50  0000 C CNN
F 2 "" H 7700 1100 50  0001 C CNN
F 3 "~" H 7700 1100 50  0001 C CNN
	1    7700 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 60B7C92E
P 2200 1150
F 0 "C3" H 2315 1196 50  0000 L CNN
F 1 "10u" H 2315 1105 50  0000 L CNN
F 2 "" H 2238 1000 50  0001 C CNN
F 3 "~" H 2200 1150 50  0001 C CNN
	1    2200 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 60B7D23A
P 5350 1250
F 0 "C6" H 5465 1296 50  0000 L CNN
F 1 "10u" H 5465 1205 50  0000 L CNN
F 2 "" H 5388 1100 50  0001 C CNN
F 3 "~" H 5350 1250 50  0001 C CNN
	1    5350 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 60B80C40
P 700 1600
F 0 "#PWR0101" H 700 1350 50  0001 C CNN
F 1 "GND" H 705 1427 50  0000 C CNN
F 2 "" H 700 1600 50  0001 C CNN
F 3 "" H 700 1600 50  0001 C CNN
	1    700  1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 60B81766
P 800 1600
F 0 "#PWR0102" H 800 1350 50  0001 C CNN
F 1 "GND" H 805 1427 50  0000 C CNN
F 2 "" H 800 1600 50  0001 C CNN
F 3 "" H 800 1600 50  0001 C CNN
	1    800  1600
	1    0    0    -1  
$EndComp
Text Label 2950 1300 2    50   ~ 0
N_BATT_CHRG
Text Label 2950 1400 2    50   ~ 0
N_BATT_STDBY
Wire Wire Line
	2950 1100 2950 1000
Connection ~ 2950 1000
$Comp
L power:GND #PWR0103
U 1 1 60B910C0
P 2200 1300
F 0 "#PWR0103" H 2200 1050 50  0001 C CNN
F 1 "GND" H 2205 1127 50  0000 C CNN
F 2 "" H 2200 1300 50  0001 C CNN
F 3 "" H 2200 1300 50  0001 C CNN
	1    2200 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 60B91771
P 3550 1700
F 0 "#PWR0104" H 3550 1450 50  0001 C CNN
F 1 "GND" H 3555 1527 50  0000 C CNN
F 2 "" H 3550 1700 50  0001 C CNN
F 3 "" H 3550 1700 50  0001 C CNN
	1    3550 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 60B9366A
P 4450 1550
F 0 "R10" H 4520 1596 50  0000 L CNN
F 1 "86k6" H 4520 1505 50  0000 L CNN
F 2 "" V 4380 1550 50  0001 C CNN
F 3 "~" H 4450 1550 50  0001 C CNN
	1    4450 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 60B93BC9
P 4450 750
F 0 "R9" H 4520 796 50  0000 L CNN
F 1 "4870" H 4520 705 50  0000 L CNN
F 2 "" V 4380 750 50  0001 C CNN
F 3 "~" H 4450 750 50  0001 C CNN
	1    4450 750 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 60B941FC
P 4150 1700
F 0 "#PWR0105" H 4150 1450 50  0001 C CNN
F 1 "GND" H 4155 1527 50  0000 C CNN
F 2 "" H 4150 1700 50  0001 C CNN
F 3 "" H 4150 1700 50  0001 C CNN
	1    4150 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 60B946E5
P 4450 1700
F 0 "#PWR0106" H 4450 1450 50  0001 C CNN
F 1 "GND" H 4455 1527 50  0000 C CNN
F 2 "" H 4450 1700 50  0001 C CNN
F 3 "" H 4450 1700 50  0001 C CNN
	1    4450 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 900  4450 1300
Wire Wire Line
	4150 1300 4450 1300
Connection ~ 4450 1300
Wire Wire Line
	4450 1300 4450 1400
Wire Wire Line
	2950 600  2950 1000
$Comp
L power:GND #PWR0107
U 1 1 60BA95CD
P 5350 1400
F 0 "#PWR0107" H 5350 1150 50  0001 C CNN
F 1 "GND" H 5355 1227 50  0000 C CNN
F 2 "" H 5350 1400 50  0001 C CNN
F 3 "" H 5350 1400 50  0001 C CNN
	1    5350 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 1100 5350 1000
Wire Wire Line
	2200 1000 2950 1000
$Comp
L Device:R R4
U 1 1 60BB3E1B
P 1850 1000
F 0 "R4" V 1643 1000 50  0000 C CNN
F 1 "0R4 1/4W" V 1734 1000 50  0000 C CNN
F 2 "" V 1780 1000 50  0001 C CNN
F 3 "~" H 1850 1000 50  0001 C CNN
	1    1850 1000
	0    1    1    0   
$EndComp
Connection ~ 2200 1000
Wire Wire Line
	2000 1000 2200 1000
Text Notes 1250 750  0    50   ~ 0
Rpog = 2k for 580mA charge current\nBatt = 750mAh\nRseries .4R @ .58A = .13W
Wire Wire Line
	1250 1200 1100 1200
Wire Wire Line
	2950 600  4450 600 
$Comp
L Device:C C7
U 1 1 60BF1C5C
P 5450 2600
F 0 "C7" V 5198 2600 50  0000 C CNN
F 1 "100n" V 5289 2600 50  0000 C CNN
F 2 "" H 5488 2450 50  0001 C CNN
F 3 "~" H 5450 2600 50  0001 C CNN
	1    5450 2600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 60BF3C95
P 5300 2600
F 0 "#PWR0108" H 5300 2350 50  0001 C CNN
F 1 "GND" V 5305 2472 50  0000 R CNN
F 2 "" H 5300 2600 50  0001 C CNN
F 3 "" H 5300 2600 50  0001 C CNN
	1    5300 2600
	0    1    1    0   
$EndComp
$Comp
L Device:C C9
U 1 1 60C07DDE
P 6450 2200
F 0 "C9" V 6198 2200 50  0000 C CNN
F 1 "100n" V 6289 2200 50  0000 C CNN
F 2 "" H 6488 2050 50  0001 C CNN
F 3 "~" H 6450 2200 50  0001 C CNN
	1    6450 2200
	0    1    1    0   
$EndComp
$Comp
L Device:C C8
U 1 1 60C08346
P 6050 2200
F 0 "C8" V 5798 2200 50  0000 C CNN
F 1 "100n" V 5889 2200 50  0000 C CNN
F 2 "" H 6088 2050 50  0001 C CNN
F 3 "~" H 6050 2200 50  0001 C CNN
	1    6050 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	6300 2200 6300 2300
$Comp
L power:GND #PWR0109
U 1 1 60C09EC3
P 5700 950
F 0 "#PWR0109" H 5700 700 50  0001 C CNN
F 1 "GND" H 5705 777 50  0000 C CNN
F 2 "" H 5700 950 50  0001 C CNN
F 3 "" H 5700 950 50  0001 C CNN
	1    5700 950 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 60C0A5B0
P 6600 2200
F 0 "#PWR0110" H 6600 1950 50  0001 C CNN
F 1 "GND" V 6605 2072 50  0000 R CNN
F 2 "" H 6600 2200 50  0001 C CNN
F 3 "" H 6600 2200 50  0001 C CNN
	1    6600 2200
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR0111
U 1 1 60C14756
P 5700 750
F 0 "#PWR0111" H 5700 600 50  0001 C CNN
F 1 "VCC" H 5717 923 50  0000 C CNN
F 2 "" H 5700 750 50  0001 C CNN
F 3 "" H 5700 750 50  0001 C CNN
	1    5700 750 
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0112
U 1 1 60C15EE1
P 6200 1550
F 0 "#PWR0112" H 6200 1400 50  0001 C CNN
F 1 "VCC" H 6217 1723 50  0000 C CNN
F 2 "" H 6200 1550 50  0001 C CNN
F 3 "" H 6200 1550 50  0001 C CNN
	1    6200 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 2300 6200 2200
Wire Wire Line
	6200 2200 6200 1850
Connection ~ 6200 2200
Wire Wire Line
	6200 1850 6300 1850
Wire Wire Line
	6300 1850 6300 2200
Connection ~ 6200 1850
Connection ~ 6300 2200
$Comp
L power:GND #PWR0113
U 1 1 60C1B1C7
P 6200 5300
F 0 "#PWR0113" H 6200 5050 50  0001 C CNN
F 1 "GND" H 6205 5127 50  0000 C CNN
F 2 "" H 6200 5300 50  0001 C CNN
F 3 "" H 6200 5300 50  0001 C CNN
	1    6200 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 60B9284D
P 4150 1550
F 0 "R8" H 4220 1596 50  0000 L CNN
F 1 "2k" H 4220 1505 50  0000 L CNN
F 2 "" V 4080 1550 50  0001 C CNN
F 3 "~" H 4150 1550 50  0001 C CNN
	1    4150 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y1
U 1 1 60C36CA8
P 7500 3250
F 0 "Y1" V 7454 3381 50  0000 L CNN
F 1 "16MHz" V 7545 3381 50  0000 L CNN
F 2 "" H 7500 3250 50  0001 C CNN
F 3 "~" H 7500 3250 50  0001 C CNN
	1    7500 3250
	0    1    1    0   
$EndComp
$Comp
L Device:C C10
U 1 1 60C37D22
P 7850 3100
F 0 "C10" V 7598 3100 50  0000 C CNN
F 1 "18p" V 7689 3100 50  0000 C CNN
F 2 "" H 7888 2950 50  0001 C CNN
F 3 "~" H 7850 3100 50  0001 C CNN
	1    7850 3100
	0    1    1    0   
$EndComp
$Comp
L Device:C C11
U 1 1 60C393A6
P 7850 3400
F 0 "C11" V 8000 3400 50  0000 C CNN
F 1 "18p" V 8100 3400 50  0000 C CNN
F 2 "" H 7888 3250 50  0001 C CNN
F 3 "~" H 7850 3400 50  0001 C CNN
	1    7850 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 3400 7500 3400
Connection ~ 7500 3400
Wire Wire Line
	7500 3400 7300 3400
Wire Wire Line
	7700 3100 7500 3100
Wire Wire Line
	7500 3100 7300 3100
Connection ~ 7500 3100
Wire Wire Line
	8000 3100 8000 3250
$Comp
L power:GND #PWR0114
U 1 1 60C41BCC
P 8000 3250
F 0 "#PWR0114" H 8000 3000 50  0001 C CNN
F 1 "GND" V 8005 3122 50  0000 R CNN
F 2 "" H 8000 3250 50  0001 C CNN
F 3 "" H 8000 3250 50  0001 C CNN
	1    8000 3250
	0    -1   -1   0   
$EndComp
Connection ~ 8000 3250
Wire Wire Line
	8000 3250 8000 3400
Wire Wire Line
	6800 3000 7200 3000
Wire Wire Line
	7200 3000 7200 2400
Wire Wire Line
	7000 2500 7000 3100
Wire Wire Line
	7000 3100 6800 3100
$Comp
L power:VCC #PWR0115
U 1 1 60C560A6
P 8800 2050
F 0 "#PWR0115" H 8800 1900 50  0001 C CNN
F 1 "VCC" H 8817 2223 50  0000 C CNN
F 2 "" H 8800 2050 50  0001 C CNN
F 3 "" H 8800 2050 50  0001 C CNN
	1    8800 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 2500 9000 2900
Wire Wire Line
	6800 2900 9000 2900
Text Notes 6400 5450 0    50   ~ 0
Where are all the ground pins?\nNOTE: Check in PCB!
$Comp
L Device:C C13
U 1 1 60C773AB
P 10250 2900
F 0 "C13" V 9998 2900 50  0000 C CNN
F 1 "220u" V 10089 2900 50  0000 C CNN
F 2 "" H 10288 2750 50  0001 C CNN
F 3 "~" H 10250 2900 50  0001 C CNN
	1    10250 2900
	0    1    1    0   
$EndComp
$Comp
L Device:C C12
U 1 1 60C7B4B4
P 10000 2250
F 0 "C12" H 9885 2204 50  0000 R CNN
F 1 "100n" H 9885 2295 50  0000 R CNN
F 2 "" H 10038 2100 50  0001 C CNN
F 3 "~" H 10000 2250 50  0001 C CNN
	1    10000 2250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 60C7C476
P 10400 3100
F 0 "#PWR0116" H 10400 2850 50  0001 C CNN
F 1 "GND" H 10405 2927 50  0000 C CNN
F 2 "" H 10400 3100 50  0001 C CNN
F 3 "" H 10400 3100 50  0001 C CNN
	1    10400 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 60C7C958
P 10000 2400
F 0 "#PWR0117" H 10000 2150 50  0001 C CNN
F 1 "GND" H 10005 2227 50  0000 C CNN
F 2 "" H 10000 2400 50  0001 C CNN
F 3 "" H 10000 2400 50  0001 C CNN
	1    10000 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 60C7E806
P 9750 3300
F 0 "#PWR0118" H 9750 3050 50  0001 C CNN
F 1 "GND" H 9755 3127 50  0000 C CNN
F 2 "" H 9750 3300 50  0001 C CNN
F 3 "" H 9750 3300 50  0001 C CNN
	1    9750 3300
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0119
U 1 1 60C7EF9A
P 10000 2100
F 0 "#PWR0119" H 10000 1950 50  0001 C CNN
F 1 "VCC" H 10017 2273 50  0000 C CNN
F 2 "" H 10000 2100 50  0001 C CNN
F 3 "" H 10000 2100 50  0001 C CNN
	1    10000 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 2100 9750 2100
Wire Wire Line
	9750 2100 9750 2500
Connection ~ 10000 2100
Wire Wire Line
	10100 2900 9750 2900
Connection ~ 9750 2900
$Comp
L Device:R R2
U 1 1 60CD8B1B
P 1200 4600
F 0 "R2" V 1400 4600 50  0000 C CNN
F 1 "100R" V 1300 4600 50  0000 C CNN
F 2 "" V 1130 4600 50  0001 C CNN
F 3 "~" H 1200 4600 50  0001 C CNN
	1    1200 4600
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 60CD638C
P 1200 4500
F 0 "R1" V 993 4500 50  0000 C CNN
F 1 "100R" V 1084 4500 50  0000 C CNN
F 2 "" V 1130 4500 50  0001 C CNN
F 3 "~" H 1200 4500 50  0001 C CNN
	1    1200 4500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 60CDDFC7
P 2250 4850
F 0 "#PWR0120" H 2250 4600 50  0001 C CNN
F 1 "GND" V 2255 4722 50  0000 R CNN
F 2 "" H 2250 4850 50  0001 C CNN
F 3 "" H 2250 4850 50  0001 C CNN
	1    2250 4850
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 4750 2250 4850
Connection ~ 2250 4850
$Comp
L power:VCC #PWR0121
U 1 1 60CE34ED
P 3950 3250
F 0 "#PWR0121" H 3950 3100 50  0001 C CNN
F 1 "VCC" H 3967 3423 50  0000 C CNN
F 2 "" H 3950 3250 50  0001 C CNN
F 3 "" H 3950 3250 50  0001 C CNN
	1    3950 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 60CE3EBE
P 3950 3700
F 0 "C5" H 3835 3654 50  0000 R CNN
F 1 "100n" H 3835 3745 50  0000 R CNN
F 2 "" H 3988 3550 50  0001 C CNN
F 3 "~" H 3950 3700 50  0001 C CNN
	1    3950 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	3500 3850 3650 3850
Connection ~ 3650 3850
$Comp
L power:GND #PWR0122
U 1 1 60CECC2C
P 3950 3850
F 0 "#PWR0122" H 3950 3600 50  0001 C CNN
F 1 "GND" H 3955 3677 50  0000 C CNN
F 2 "" H 3950 3850 50  0001 C CNN
F 3 "" H 3950 3850 50  0001 C CNN
	1    3950 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 60CF2B8C
P 2850 3850
F 0 "R6" V 2643 3850 50  0000 C CNN
F 1 "20k" V 2734 3850 50  0000 C CNN
F 2 "" V 2780 3850 50  0001 C CNN
F 3 "~" H 2850 3850 50  0001 C CNN
	1    2850 3850
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 60CF333A
P 2550 3850
F 0 "R5" V 2343 3850 50  0000 C CNN
F 1 "18k" V 2434 3850 50  0000 C CNN
F 2 "" V 2480 3850 50  0001 C CNN
F 3 "~" H 2550 3850 50  0001 C CNN
	1    2550 3850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0123
U 1 1 60D0AA3A
P 2400 3850
F 0 "#PWR0123" H 2400 3600 50  0001 C CNN
F 1 "GND" V 2405 3722 50  0000 R CNN
F 2 "" H 2400 3850 50  0001 C CNN
F 3 "" H 2400 3850 50  0001 C CNN
	1    2400 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	1950 4500 2250 4500
Wire Wire Line
	1950 4600 2250 4600
$Comp
L power:GND #PWR0124
U 1 1 60D1703A
P 2800 5350
F 0 "#PWR0124" H 2800 5100 50  0001 C CNN
F 1 "GND" H 2805 5177 50  0000 C CNN
F 2 "" H 2800 5350 50  0001 C CNN
F 3 "" H 2800 5350 50  0001 C CNN
	1    2800 5350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 60D19B2E
P 3350 5350
F 0 "#PWR0125" H 3350 5100 50  0001 C CNN
F 1 "GND" H 3355 5177 50  0000 C CNN
F 2 "" H 3350 5350 50  0001 C CNN
F 3 "" H 3350 5350 50  0001 C CNN
	1    3350 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 60D1A214
P 2200 5200
F 0 "C4" H 2085 5154 50  0000 R CNN
F 1 "100n" H 2085 5245 50  0000 R CNN
F 2 "" H 2238 5050 50  0001 C CNN
F 3 "~" H 2200 5200 50  0001 C CNN
	1    2200 5200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0126
U 1 1 60D1A8BB
P 2200 5350
F 0 "#PWR0126" H 2200 5100 50  0001 C CNN
F 1 "GND" H 2205 5177 50  0000 C CNN
F 2 "" H 2200 5350 50  0001 C CNN
F 3 "" H 2200 5350 50  0001 C CNN
	1    2200 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 5050 2250 5050
Wire Wire Line
	3000 3850 3000 3550
Connection ~ 2700 3850
Wire Wire Line
	2250 4350 2100 4350
Wire Wire Line
	2100 4350 2100 3550
Connection ~ 3000 3550
Wire Wire Line
	3400 3550 3650 3550
Connection ~ 3650 3550
Wire Wire Line
	3650 3550 3650 3850
Wire Wire Line
	3950 3550 3650 3550
Connection ~ 3950 3550
$Comp
L Device:C C1
U 1 1 60D35911
P 1550 3700
F 0 "C1" H 1665 3746 50  0000 L CNN
F 1 "10u" H 1665 3655 50  0000 L CNN
F 2 "" H 1588 3550 50  0001 C CNN
F 3 "~" H 1550 3700 50  0001 C CNN
	1    1550 3700
	1    0    0    -1  
$EndComp
Connection ~ 2100 3550
$Comp
L power:GND #PWR0127
U 1 1 60D39045
P 1550 3850
F 0 "#PWR0127" H 1550 3600 50  0001 C CNN
F 1 "GND" H 1555 3677 50  0000 C CNN
F 2 "" H 1550 3850 50  0001 C CNN
F 3 "" H 1550 3850 50  0001 C CNN
	1    1550 3850
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 60D3CAA5
P 850 4500
F 0 "J2" H 850 4700 50  0000 C CNN
F 1 "Load cell" V 950 4450 50  0000 C CNN
F 2 "" H 850 4500 50  0001 C CNN
F 3 "~" H 850 4500 50  0001 C CNN
	1    850  4500
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 60D4BE2C
P 1050 4700
F 0 "#PWR0128" H 1050 4450 50  0001 C CNN
F 1 "GND" H 1055 4527 50  0000 C CNN
F 2 "" H 1050 4700 50  0001 C CNN
F 3 "" H 1050 4700 50  0001 C CNN
	1    1050 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 4400 1050 3550
Wire Wire Line
	2100 3550 3000 3550
$Comp
L Device:R R7
U 1 1 60D4FBA6
P 3950 3400
F 0 "R7" H 3880 3354 50  0000 R CNN
F 1 "0R" H 3880 3445 50  0000 R CNN
F 2 "" V 3880 3400 50  0001 C CNN
F 3 "~" H 3950 3400 50  0001 C CNN
	1    3950 3400
	-1   0    0    1   
$EndComp
$Comp
L Device:R R12
U 1 1 60D63711
P 6200 1700
F 0 "R12" H 6130 1654 50  0000 R CNN
F 1 "0R" H 6130 1745 50  0000 R CNN
F 2 "" V 6130 1700 50  0001 C CNN
F 3 "~" H 6200 1700 50  0001 C CNN
	1    6200 1700
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J6
U 1 1 60D6D08D
P 8700 4300
F 0 "J6" H 8700 4500 50  0000 C CNN
F 1 "UART" V 8800 4250 50  0000 C CNN
F 2 "" H 8700 4300 50  0001 C CNN
F 3 "~" H 8700 4300 50  0001 C CNN
	1    8700 4300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 60D71D1F
P 8500 4500
F 0 "#PWR0129" H 8500 4250 50  0001 C CNN
F 1 "GND" H 8505 4327 50  0000 C CNN
F 2 "" H 8500 4500 50  0001 C CNN
F 3 "" H 8500 4500 50  0001 C CNN
	1    8500 4500
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0130
U 1 1 60D728CC
P 8500 4200
F 0 "#PWR0130" H 8500 4050 50  0001 C CNN
F 1 "VCC" H 8517 4373 50  0000 C CNN
F 2 "" H 8500 4200 50  0001 C CNN
F 3 "" H 8500 4200 50  0001 C CNN
	1    8500 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 4300 6800 4300
Wire Wire Line
	6800 4400 8500 4400
Text Label 6800 4700 0    50   ~ 0
LED_A
Text Label 7400 1100 2    50   ~ 0
LED_A
Text Label 6800 4800 0    50   ~ 0
LED_B
$Comp
L Device:R R13
U 1 1 60DAE3A0
P 8150 1100
F 0 "R13" V 7943 1100 50  0000 C CNN
F 1 "330" V 8034 1100 50  0000 C CNN
F 2 "" V 8080 1100 50  0001 C CNN
F 3 "~" H 8150 1100 50  0001 C CNN
	1    8150 1100
	0    1    1    0   
$EndComp
Text Label 8300 1100 0    50   ~ 0
LED_B
$Comp
L Device:R R11
U 1 1 60DC0B90
P 4700 1000
F 0 "R11" V 4800 1000 50  0000 C CNN
F 1 "0R" V 4600 1000 50  0000 C CNN
F 2 "" V 4630 1000 50  0001 C CNN
F 3 "~" H 4700 1000 50  0001 C CNN
	1    4700 1000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4550 1000 4150 1000
Connection ~ 5350 1000
Text Label 4050 4500 0    50   ~ 0
SCK
Text Label 4050 4600 0    50   ~ 0
DOUT
Text Label 4050 5000 0    50   ~ 0
RATE
Text Label 6800 2800 0    50   ~ 0
SCK
Text Label 6800 4500 0    50   ~ 0
DOUT
Text Label 6800 4600 0    50   ~ 0
RATE
$Comp
L Device:R R15
U 1 1 60DE0A0E
P 8600 3350
F 0 "R15" H 8670 3396 50  0000 L CNN
F 1 "1M" H 8670 3305 50  0000 L CNN
F 2 "" V 8530 3350 50  0001 C CNN
F 3 "~" H 8600 3350 50  0001 C CNN
	1    8600 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 60DE1112
P 8600 3650
F 0 "R16" H 8670 3696 50  0000 L CNN
F 1 "220k" H 8670 3605 50  0000 L CNN
F 2 "" V 8530 3650 50  0001 C CNN
F 3 "~" H 8600 3650 50  0001 C CNN
	1    8600 3650
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0131
U 1 1 60DE264C
P 8600 3200
F 0 "#PWR0131" H 8600 3050 50  0001 C CNN
F 1 "VCC" H 8617 3373 50  0000 C CNN
F 2 "" H 8600 3200 50  0001 C CNN
F 3 "" H 8600 3200 50  0001 C CNN
	1    8600 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 3600 7250 3600
Wire Wire Line
	7250 3600 7250 3800
Wire Wire Line
	6800 3500 8600 3500
Connection ~ 8600 3500
Text Label 6800 5000 0    50   ~ 0
N_BATT_STDBY
Text Label 6800 4900 0    50   ~ 0
N_BATT_CHRG
Wire Wire Line
	7300 3100 7300 3200
Wire Wire Line
	7300 3400 7300 3300
Wire Wire Line
	6800 3300 7300 3300
Wire Wire Line
	6800 3200 7300 3200
$Comp
L Device:R PTC1
U 1 1 60E3219C
P 4850 1550
F 0 "PTC1" H 4920 1596 50  0000 L CNN
F 1 "NTCG103UH103JT1" H 4900 1400 50  0000 L CNN
F 2 "" V 4780 1550 50  0001 C CNN
F 3 "~" H 4850 1550 50  0001 C CNN
	1    4850 1550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 60E34E90
P 4850 1700
F 0 "#PWR0132" H 4850 1450 50  0001 C CNN
F 1 "GND" H 4855 1527 50  0000 C CNN
F 2 "" H 4850 1700 50  0001 C CNN
F 3 "" H 4850 1700 50  0001 C CNN
	1    4850 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 60E7358C
P 1550 4550
F 0 "C2" H 1435 4504 50  0000 R CNN
F 1 "100n" H 1435 4595 50  0000 R CNN
F 2 "" H 1588 4400 50  0001 C CNN
F 3 "~" H 1550 4550 50  0001 C CNN
	1    1550 4550
	-1   0    0    1   
$EndComp
Wire Wire Line
	1950 4500 1950 4400
Wire Wire Line
	1950 4400 1550 4400
Wire Wire Line
	1350 4400 1350 4500
Connection ~ 1550 4400
Wire Wire Line
	1550 4400 1350 4400
Wire Wire Line
	1350 4600 1350 4700
Wire Wire Line
	1350 4700 1550 4700
Wire Wire Line
	1950 4700 1950 4600
Connection ~ 1550 4700
Wire Wire Line
	1550 4700 1950 4700
Connection ~ 1550 3550
Wire Wire Line
	1550 3550 2100 3550
Wire Wire Line
	1050 3550 1550 3550
Wire Wire Line
	9450 2700 9450 2900
Connection ~ 9000 2900
Connection ~ 9450 2900
Wire Wire Line
	9450 2900 9450 3100
Wire Wire Line
	4850 1300 4850 1400
Wire Wire Line
	4450 1300 4850 1300
Wire Wire Line
	4850 1000 5350 1000
$Comp
L Connector_Generic:Conn_01x01 J7
U 1 1 60F2C3CD
P 10600 2900
F 0 "J7" H 10680 2942 50  0000 L CNN
F 1 "Spk+" H 10680 2851 50  0000 L CNN
F 2 "" H 10600 2900 50  0001 C CNN
F 3 "~" H 10600 2900 50  0001 C CNN
	1    10600 2900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J8
U 1 1 60F32235
P 10600 3100
F 0 "J8" H 10680 3142 50  0000 L CNN
F 1 "Spk-" H 10680 3051 50  0000 L CNN
F 2 "" H 10600 3100 50  0001 C CNN
F 3 "~" H 10600 3100 50  0001 C CNN
	1    10600 3100
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J3
U 1 1 60F3305D
P 5900 750
F 0 "J3" H 5980 792 50  0000 L CNN
F 1 "Batt+" H 5980 701 50  0000 L CNN
F 2 "" H 5900 750 50  0001 C CNN
F 3 "~" H 5900 750 50  0001 C CNN
	1    5900 750 
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J4
U 1 1 60F3415B
P 5900 950
F 0 "J4" H 5980 992 50  0000 L CNN
F 1 "Batt-" H 5980 901 50  0000 L CNN
F 2 "" H 5900 950 50  0001 C CNN
F 3 "~" H 5900 950 50  0001 C CNN
	1    5900 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 750  5350 1000
Connection ~ 5700 750 
$Comp
L Device:R R3
U 1 1 60F7770F
P 1400 1200
F 0 "R3" V 1300 1050 50  0000 C CNN
F 1 "200" V 1284 1200 50  0000 C CNN
F 2 "" V 1330 1200 50  0001 C CNN
F 3 "~" H 1400 1200 50  0001 C CNN
	1    1400 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	1550 1200 1550 1300
Wire Wire Line
	1100 1300 1550 1300
Wire Wire Line
	9350 2900 9450 2900
Wire Wire Line
	9050 2900 9000 2900
$Comp
L Device:R R17
U 1 1 60C85088
P 9200 2900
F 0 "R17" V 8993 2900 50  0000 C CNN
F 1 "1k" V 9084 2900 50  0000 C CNN
F 2 "" V 9130 2900 50  0001 C CNN
F 3 "~" H 9200 2900 50  0001 C CNN
	1    9200 2900
	0    1    1    0   
$EndComp
$Comp
L Device:R R14
U 1 1 60B884C0
P 8550 2050
F 0 "R14" V 8343 2050 50  0000 C CNN
F 1 "10k" V 8434 2050 50  0000 C CNN
F 2 "" V 8480 2050 50  0001 C CNN
F 3 "~" H 8550 2050 50  0001 C CNN
	1    8550 2050
	0    1    1    0   
$EndComp
Wire Wire Line
	8700 2050 8800 2050
Connection ~ 8800 2050
Wire Wire Line
	8400 2050 8250 2050
Wire Wire Line
	8250 2050 8250 2600
Wire Wire Line
	8250 4100 6800 4100
Connection ~ 8250 2600
Wire Wire Line
	8250 2600 8250 4100
Wire Wire Line
	7250 3800 8600 3800
$Comp
L power:GND #PWR0133
U 1 1 60C5568D
P 8800 2600
F 0 "#PWR0133" H 8800 2350 50  0001 C CNN
F 1 "GND" H 8805 2427 50  0000 C CNN
F 2 "" H 8800 2600 50  0001 C CNN
F 3 "" H 8800 2600 50  0001 C CNN
	1    8800 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 2600 8250 2600
Wire Wire Line
	8800 2500 9000 2500
Wire Wire Line
	7000 2500 8300 2500
Wire Wire Line
	8800 2400 8800 2050
Wire Wire Line
	7200 2400 8300 2400
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J5
U 1 1 60C43461
P 8500 2500
F 0 "J5" H 8550 2300 50  0000 C CNN
F 1 "ICSP" H 8550 2726 50  0000 C CNN
F 2 "" H 8500 2500 50  0001 C CNN
F 3 "~" H 8500 2500 50  0001 C CNN
	1    8500 2500
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 60BEB752
P 3650 3550
F 0 "#FLG0101" H 3650 3625 50  0001 C CNN
F 1 "PWR_FLAG" H 3650 3723 50  0000 C CNN
F 2 "" H 3650 3550 50  0001 C CNN
F 3 "~" H 3650 3550 50  0001 C CNN
	1    3650 3550
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 60BED3FB
P 6550 1550
F 0 "#FLG0102" H 6550 1625 50  0001 C CNN
F 1 "PWR_FLAG" H 6550 1723 50  0000 C CNN
F 2 "" H 6550 1550 50  0001 C CNN
F 3 "~" H 6550 1550 50  0001 C CNN
	1    6550 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 1550 6550 1850
Wire Wire Line
	6550 1850 6300 1850
Connection ~ 6300 1850
Wire Wire Line
	5350 750  5700 750 
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 60BF3AF3
P 5350 750
F 0 "#FLG0103" H 5350 825 50  0001 C CNN
F 1 "PWR_FLAG" H 5350 923 50  0000 C CNN
F 2 "" H 5350 750 50  0001 C CNN
F 3 "~" H 5350 750 50  0001 C CNN
	1    5350 750 
	1    0    0    -1  
$EndComp
Connection ~ 5350 750 
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 60BF606A
P 2200 1000
F 0 "#FLG0104" H 2200 1075 50  0001 C CNN
F 1 "PWR_FLAG" H 2200 1173 50  0000 C CNN
F 2 "" H 2200 1000 50  0001 C CNN
F 3 "~" H 2200 1000 50  0001 C CNN
	1    2200 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 1000 1700 1000
$Comp
L power:PWR_FLAG #FLG0105
U 1 1 60C03BBA
P 1550 3550
F 0 "#FLG0105" H 1550 3625 50  0001 C CNN
F 1 "PWR_FLAG" H 1550 3723 50  0000 C CNN
F 2 "" H 1550 3550 50  0001 C CNN
F 3 "~" H 1550 3550 50  0001 C CNN
	1    1550 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60C0FDEA
P 5900 2200
F 0 "#PWR?" H 5900 1950 50  0001 C CNN
F 1 "GND" V 5905 2072 50  0000 R CNN
F 2 "" H 5900 2200 50  0001 C CNN
F 3 "" H 5900 2200 50  0001 C CNN
	1    5900 2200
	0    1    1    0   
$EndComp
$EndSCHEMATC
