$ErrorActionPreference = 'Stop'
$xlsx = 'c:\Users\Administrator\Desktop\U1U4U7\pc_format.xlsx'
$tmp = Join-Path $env:TEMP "xlsx_$(Get-Random)"
New-Item -ItemType Directory -Path $tmp | Out-Null
Add-Type -AssemblyName System.IO.Compression.FileSystem
[System.IO.Compression.ZipFile]::ExtractToDirectory($xlsx, $tmp)

# 读取共享字符串
$shared = @()
$sstPath = Join-Path $tmp 'xl\sharedStrings.xml'
if (Test-Path $sstPath) {
    [xml]$sstXml = Get-Content $sstPath -Encoding UTF8
    foreach ($si in $sstXml.sst.si) {
        $text = ($si.t | ForEach-Object { $_.InnerText }) -join ''
        $shared += $text
    }
}

# 读取所有 sheet
$workbookPath = Join-Path $tmp 'xl\workbook.xml'
[xml]$wbXml = Get-Content $workbookPath -Encoding UTF8
$sheetIndex = 0
foreach ($sheet in $wbXml.workbook.sheets.sheet) {
    $sheetIndex++
    $sheetFile = Join-Path $tmp "xl\worksheets\sheet$sheetIndex.xml"
    if (-not (Test-Path $sheetFile)) { continue }
    [xml]$sheetXml = Get-Content $sheetFile -Encoding UTF8
    Write-Host "=== Sheet: $($sheet.name) ==="
    foreach ($row in $sheetXml.worksheet.sheetData.row) {
        $cells = @()
        foreach ($c in $row.c) {
            $val = ''
            if ($c.t -eq 's' -and $c.v -ne $null) {
                $idx = [int]$c.v
                if ($idx -lt $shared.Count) { $val = $shared[$idx] }
            } elseif ($c.t -eq 'inlineStr') {
                $val = $c.is.t
            } else {
                $val = $c.v
            }
            if ($val -eq $null) { $val = '' }
            $cells += "$($c.r)=$val"
        }
        Write-Host ($cells -join ' | ')
    }
}

Remove-Item -Recurse -Force $tmp
