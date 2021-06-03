On Error Resume Next
Dim isOccupied, port
isOccupied = 0
port = 9876
Dim processNum       '定义变量'
processNum=0        '变量初始化赋值'
Dim processWebNum       '定义变量'
processWebNum=0 

Set WshShell = WScript.CreateObject("WScript.Shell")
' Set oExec = WshShell.Exec("netstat -an",0)
' Set oStdOut = oExec.StdOut
' Dim path			'定义地址变量'
' path = createobject("Scripting.FileSystemObject").GetFile(Wscript.ScriptFullName).ParentFolder.Path+"\SerialPort.exe"
' strComputer = "."
Set objWMIService = GetObject("winmgmts:" _
& "{impersonationLevel=impersonate}!\\" & strComputer & "\root\cimv2")
Set colProcessList = objWMIService.ExecQuery _
("Select * from Win32_Process Where Name = 'SerialPort.exe'")
Set colProcessListweb = objWMIService.ExecQuery _
("Select * from Win32_Process Where Name = 'PasuoWebServer.exe'")
For Each objProcess in colProcessList
' if objProcess.executablepath = path then
    if objProcess.Name = "SerialPort.exe" then
        processNum = processNum + 1
    end if
Next

For Each objProcessx in colProcessListweb
    if objProcessx.Name = "PasuoWebServer.exe" then
        processWebNum = processWebNum + 10
    end if
Next
' Wscript.Echo processNum
' Wscript.Echo processWebNum
if processNum<>0 then
    ' Dim WinScriptHost
    WshShell.run "cmd.exe /C Taskkill /f /im SerialPort.exe",0
    ' Set WinScriptHost = Nothing
    WScript.Sleep(1000)
end if
WshShell.run "cmd /c SerialPort.exe",1
if processWebNum=0 then
    WshShell.run "cmd /c PasuoWebServer.exe",1
end if
' Do Until oStdOut.AtEndOfStream
' strLine = oStdOut.ReadLine
' If InStr(strLine, ":" & port) > 0 And InStrRev(strLine, "LISTENING") > 0 Then
' ' WshShell.run "cmd /c java -jar climb-client-jfx.jar",0
' wscript.quit
' End If
' Loop
' WshShell.run "cmd /c PasuoWebServer.exe",1
WshShell.run "cmd /c java -jar climb-client-jfx.jar",0