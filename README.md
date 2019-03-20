Port of the espresso ESP32 Bluetooth A2DP_Source to compile and log to Serial using Arduino
https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/a2dp_source
<p>
The following link is another modification of the espressif source (which I found after uploading this):
https://github.com/markingle/a2dp_source
<p>
I have found the esp-idf can be slow and/or require multiple reboot of ESP and/or bt headset before it will pair, so I may also try the Blue Kitchen btstack https://github.com/bluekitchen/btstack which has a port for ESP-32 and rumour has it :-p might be more stable...
<p>
Ideally this could be merged with a more complete library which includes decoding of other codecs, etc
<p><p>

As it stands the code can connect to my BNX-60 bluetooth headset and play white noise or one of a couple of other patterns as per the original example, but playing from a proper audio file isn't implemented...
<p><p><p>



BTW in case it's helpful the following script can be run in Microsoft Word to convert binary files (such as mods or wav files) into text format for use with progmem if using eg ESP8622Audio library
https://github.com/earlephilhower/ESP8266Audio

```
Sub modsToText()
    curFilename = "C:\Users\dgm3333\Downloads\1_channel_moog.it"      ' change this to your file name

    Dim intFileNum%, bytTemp As Byte, intCellRow%
    intFileNum = FreeFile
    intCellRow = 0
    Open curFilename For Binary Access Read As intFileNum
    linecount = 0
    Selection.TypeText ("const unsigned char converted_mod[] PROGMEM = {" & vbCrLf & " ")
    Do While Not EOF(intFileNum)
        Get intFileNum, , bytTemp
        'Selection.End.Select
        a = Hex(bytTemp)
        If Len(a) = 1 Then a = "0" & a
        Selection.TypeText (" 0x" & a)
        If Not EOF(intFileNum) Then
            linecount = linecount + 1
            If linecount = 9 Then linecount = 0
            If linecount = 0 Then
                Selection.TypeText ("," & vbCrLf & " ")
            Else
                Selection.TypeText (",")
            End If
        End If
    Loop
    Selection.TypeText (vbCrLf & "};")

    Close intFileNum
End Sub
```
<p>


