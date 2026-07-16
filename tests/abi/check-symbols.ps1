param(
    [Parameter(Mandatory = $true)][string]$Library,
    [Parameter(Mandatory = $true)][string]$Baseline
)
$expected = @((Get-Content -LiteralPath $Baseline -Raw | ConvertFrom-Json).exports | Sort-Object)
$names = @(llvm-readobj --coff-exports $Library | Select-String '^\s+Name:\s+' | ForEach-Object {
    ($_.Line -replace '^\s+Name:\s+', '').Trim()
})
if ($LASTEXITCODE -ne 0) { throw "llvm-readobj failed for $Library" }
$platform = @('_DllMainCRTStartup', '__xl_a', '__xl_z', '_tls_end', '_tls_index', '_tls_start', '_tls_used')
$actual = @($names | Where-Object { $_ -notin $platform } | Sort-Object)
$difference = Compare-Object -ReferenceObject $expected -DifferenceObject $actual
if ($difference) {
    $difference | Format-Table | Out-String | Write-Error
    throw 'Gravity C ABI export allowlist mismatch.'
}
