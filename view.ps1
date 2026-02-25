$keepRootFiles = @(".gitignore", "CMakeLists.txt", "gurobi_log.txt", "run.bat")
$keepCoreDirs = @("src")
$symbolBranch = "©À©¤"
$symbolLast = "©¸©¤"
$symbolLine = "©¦"
$symbolSpace = "  "

function Get-TreeWithStructure {
    param(
        [string]$CurrentPath = ".",
        [string]$Prefix = ""
    )

    $files = Get-ChildItem -Path $CurrentPath -File | Where-Object {
        ($CurrentPath -eq "." -and $keepRootFiles -contains $_.Name) -or
        ($CurrentPath -like "*src*" -and $_.Extension -in ".cpp", ".hpp", ".txt", ".cmake", ".tex")
    } | Sort-Object Name

    $dirs = Get-ChildItem -Path $CurrentPath -Directory | Where-Object {
        $keepCoreDirs -contains $_.Name -or $CurrentPath -like "*src*"
    } | Sort-Object Name

    $totalItems = $files.Count + $dirs.Count
    $currentIndex = 0

    foreach ($file in $files) {
        $currentIndex++
        $isLast = $currentIndex -eq $totalItems
        $sym = if ($isLast) { $symbolLast } else { $symbolBranch }
        Write-Host "$Prefix$sym$($file.Name)"
    }

    foreach ($dir in $dirs) {
        $currentIndex++
        $isLast = $currentIndex -eq $totalItems
        $sym = if ($isLast) { $symbolLast } else { $symbolBranch }
        Write-Host "$Prefix$sym$($dir.Name)"

        $newPrefix = if ($isLast) {
            "$Prefix$symbolSpace"
        } else {
            "$Prefix$symbolLine$symbolSpace"
        }

        Get-TreeWithStructure -CurrentPath $dir.FullName -Prefix $newPrefix
    }
}

Write-Host "C:."
Get-TreeWithStructure -CurrentPath "." -Prefix ""