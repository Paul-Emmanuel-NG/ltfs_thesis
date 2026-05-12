$params = @(
    @{name="theta_s"; flag="--theta-s"; values=@(3.0, 5.0, 7.0)},
    @{name="theta_u_s"; flag="--theta-u-s"; values=@(1.5, 3.0, 4.5)}
)

foreach ($p in $params) {
    foreach ($val in $p.values) {
        $vstr = ($val.ToString()).Replace(".", "p")
        $dir = "results_urgency_$($p.name)_$vstr"
        mkdir $dir -ErrorAction SilentlyContinue
        foreach ($seed in 1,2,3,4,5,6,7,8,9,10) {
            Write-Host "=== A8 S1 seed $seed $($p.flag)=$val (dir=$dir) ==="
            python -m ltfs.runner --variant A8 --scenario S1 --seed $seed --sim-time 3600 $p.flag $val --sumo-cfg osm_light.sumocfg --quiet --results-dir $dir
        }
    }
}