$bbox_overpass = "31.218240,121.447363,31.233231,121.481675"
$workdir = "C:\work\thesis\verification_2026-04-27"
$sumohome = $env:SUMO_HOME

New-Item -ItemType Directory -Path $workdir -Force | Out-Null
Set-Location $workdir

# === Step 1: Download OSM (fixed) ===
Write-Host "=== Step 1: Downloading OSM data ==="
$query = @"
[out:xml][timeout:300];
(node($bbox_overpass);way($bbox_overpass);relation($bbox_overpass););
out body;
>;
out skel qt;
"@

# URL-encode the query and prepend "data="
Add-Type -AssemblyName System.Web
$encoded = [System.Web.HttpUtility]::UrlEncode($query)
$body = "data=$encoded"

Invoke-WebRequest -Uri "https://overpass-api.de/api/interpreter" `
    -Method POST `
    -Body $body `
    -ContentType "application/x-www-form-urlencoded" `
    -OutFile "osm_data.osm"

$size = (Get-Item osm_data.osm).Length
Write-Host "  Downloaded osm_data.osm ($size bytes)"
if ($size -lt 100000) {
    Write-Host "  WARNING: file too small, check error"
    Get-Content osm_data.osm | Select-Object -First 20
    exit 1
}

# === Step 2: Build network ===
Write-Host "=== Step 2: Running netconvert ==="
$netconvert = "$sumohome\bin\netconvert.exe"
& $netconvert `
    --osm-files osm_data.osm `
    --output-file yanan_regenerated.net.xml `
    --geometry.remove `
    --ramps.guess `
    --junctions.join `
    --tls.guess-signals `
    --tls.discard-simple `
    --tls.join `
    --osm.layer-elevation 6.0 `
    --output.original-names

if (-not (Test-Path "yanan_regenerated.net.xml")) {
    Write-Host "ERROR: netconvert failed"
    exit 1
}
Write-Host "  Network created"

# === Step 3: Generate trips ===
Write-Host "=== Step 3: Generating trips ==="
$randomtrips = "$sumohome\tools\randomTrips.py"

& python $randomtrips -n yanan_regenerated.net.xml -r passenger.rou.xml -o passenger.trips.xml -e 3600 --vehicle-class passenger --prefix "veh_" --validate
& python $randomtrips -n yanan_regenerated.net.xml -r bus.rou.xml -o bus.trips.xml -e 3600 --vehicle-class bus --prefix "bus_" --period 6 --validate
& python $randomtrips -n yanan_regenerated.net.xml -r truck.rou.xml -o truck.trips.xml -e 3600 --vehicle-class truck --prefix "truck_" --period 12 --validate

Write-Host "=== DONE ==="
Get-ChildItem | Format-Table Name, Length, LastWriteTime
