@Echo Off
Echo Sync Running Show (L: Wizlights) back to Active Development (W:)
pause
robocopy L:\Documents\*.* W:\!ActiveDocuments\Christmas\Documents /S/E/Z/MT:6/XO/R:4/TBD
robocopy L:\xLights\2019\WizLights\*.* W:\!ActiveDocuments\Christmas\xLights\2019\WizLights /d/c/i/f/h/k/y
robocopy L:\xLights\2019\WizLights\Sequences\*.* W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Sequences /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy L:\xLights\2019\WizLights\Audio\*.* W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Audio /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy L:\xLights\2019\WizLights\Images\*.* W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Images /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy L:\xLights\2019\WizLights\Models\*.* W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Models /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy L:\xLights\2019\WizLights\Palettes\*.* W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Palettes /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy L:\xLights\2019\WizLights\RenderCache\*.* W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\RenderCache /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy L:\xLights\2019\WizLights\xScheduleData\*.* W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\xScheduleData /S/E/Z//XO/IA/V/MT:6/R:4/TBD
Echo
Echo Sync Active Development (W:) to Running Show (L: Wizlights)
pause
robocopy W:\!ActiveDocuments\Christmas\Documents\*.* L:\Documents /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\*.* L:\xLights\2019\WizLights /d/c/i/f/h/k/y
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Sequences\*.* L:\xLights\2019\WizLights\Sequences /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Audio\*.* L:\xLights\2019\WizLights\Audio /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Images\*.* L:\xLights\2019\WizLights\Images /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Models\*.* L:\xLights\2019\WizLights\Models /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Palettes\*.* L:\xLights\2019\WizLights\Palettes /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\RenderCache\*.* L:\xLights\2019\WizLights\RenderCache /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\xScheduleData\*.* L:\xLights\2019\WizLights\xScheduleData /S/E/Z//XO/IA/V/MT:6/R:4/TBD
Echo
Echo Sync Active Development (W:) to Archival (D:)
pause
robocopy W:\!ActiveDocuments\Christmas\Documents\*.* D:\Christmas\Documents /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\*.* D:\Christmas\xLights\2019\WizLights /d/c/i/f/h/k/y
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Sequences\*.* D:\Christmas\xLights\2019\WizLights\Sequences /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Audio\*.* D:\Christmas\xLights\2019\WizLights\Audio /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Images\*.* D:\Christmas\xLights\2019\WizLights\Images /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Models\*.* D:\Christmas\xLights\2019\WizLights\Models /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Palettes\*.* D:\Christmas\xLights\2019\WizLights\Palettes /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\RenderCache\*.* D:\Christmas\xLights\2019\WizLights\RenderCache /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\xScheduleData\*.* D:\Christmas\xLights\2019\WizLights\xScheduleData /S/E/Z//XO/IA/V/MT:6/R:4/TBD
Echo
Echo Sync Active Development (W:) to External Backup (Z: WizNaz)
pause
robocopy W:\!ActiveDocuments\Christmas\Documents\*.* Z:\Christmas\Documents /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\*.* Z:\Christmas\xLights\2019\WizLights /d/c/i/f/h/k/y
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Sequences\*.* Z:\Christmas\xLights\2019\WizLights\Sequences /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Audio\*.* Z:\Christmas\xLights\2019\WizLights\Audio /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Images\*.* Z:\Christmas\xLights\2019\WizLights\Images /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Models\*.* Z:\Christmas\xLights\2019\WizLights\Models /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\Palettes\*.* Z:\Christmas\xLights\2019\WizLights\Palettes /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\RenderCache\*.* Z:\Christmas\xLights\2019\WizLights\RenderCache /S/E/Z//XO/IA/V/MT:6/R:4/TBD
robocopy W:\!ActiveDocuments\Christmas\xLights\2019\WizLights\xScheduleData\*.* Z:\Christmas\xLights\2019\WizLights\xScheduleData /S/E/Z//XO/IA/V/MT:6/R:4/TBD
Echo Sync Complete between W:, L:, D: and Z:
pause
