In diesem Ordner sind die Sourcen fuer die Converter enthalten

Folders:
* AADC_WheelRpmConverter:	Mit diesem Filter k�nnen die Raddrehzahlen der R�der berechnet werden. Dabei wird ein Sliding-Window-Filter verwendet der aus den �bergebenen Z�hlerwerten die Raddrehzahlwerte berechnet.
* AADC_AttitudeConverter:	Dieser Filter konvertiert die Lage des Fahrzeugs von Quaternionen zu Euler Winkel.
* AADC_Calibration:	Mit diesem Filter k�nnen Sensor- oder Aktorwerte linear skaliert werden. Dabei wird der Eingangswert  mit dem in den Eigenschaften gegebenen Wert multipliziert. 
* AADC_CalibrationXml:	Dieser Filter dient zur Kalibrierung und Skalierung von Daten. Im Gegensatz zum Calibration Scaling Filter wird hier eine abschnittsweise definierte Funktion f(x) mit St�tzstellen zur Interpolation der Messdaten verwendet	


Files:
* CMakeLists.txt:	Datei der CMake kette
* Readme.txt:		Diese Datei