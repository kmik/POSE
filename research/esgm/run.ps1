param(
    [ValidateSet('small','large','adaptive','pair','replay')][string]$Mode='small',
    [int]$Gpu=0,
    [string]$Output='',
    [Parameter(ValueFromRemainingArguments=$true)][string[]]$ResearchArgs=@()
)
$ErrorActionPreference='Stop'
$module=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
Push-Location $module
try {
    if (-not (Test-Path -LiteralPath 'lib/opencv-4.9.0-0.jar')) {
        throw 'Dependencies missing. Run build.bat once before the research tools.'
    }
    $runtime='java';$compiler='javac'
    $jdk=$env:JAVA_HOME
    if (-not $jdk) {
        $portable=Get-ChildItem tools -Directory -Filter 'jdk-25*' -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($portable) {$jdk=$portable.FullName}
    }
    if ($jdk) {$runtime=Join-Path $jdk 'bin/java.exe';$compiler=Join-Path $jdk 'bin/javac.exe'}
    $classes=Join-Path $module 'target/esgm-research-tool/classes'
    New-Item -ItemType Directory -Force $classes | Out-Null
    $sources=@(Get-ChildItem src/main/java -Recurse -Filter '*.java' | ForEach-Object {$_.FullName})
    foreach($name in @('TestSupport','EsgmPrototype','EsgmInvestigation','AdaptiveStereoInvestigation','AdaptiveSearchBenchmark','RectifiedResearchReplay')) {
        $sources+=(Join-Path $module "src/test/java/sgm/$name.java")
    }
    & $compiler --release 25 -encoding UTF-8 -cp 'lib/*' -d $classes $sources
    if ($LASTEXITCODE -ne 0) {throw 'Research compilation failed'}
    if (-not $Output) {$Output=Join-Path $module ('target/esgm-research/'+$Mode+'-'+(Get-Date -Format 'yyyyMMddTHHmmssfff'))}
    $main='sgm.EsgmInvestigation';$caseArgs=@()
    switch($Mode) {
        'large' {$caseArgs=@('large')}
        'adaptive' {$main='sgm.AdaptiveSearchBenchmark'}
        'pair' {$caseArgs=@('pair')}
        'replay' {$main='sgm.RectifiedResearchReplay'}
    }
    $heap=$env:ESGM_RESEARCH_HEAP;if (-not $heap) {$heap='4g'}
    & $runtime --enable-native-access=ALL-UNNAMED "-Xmx$heap" "-Dsgm.test.gpu=$Gpu" "-Dsgm.research.output=$Output" -cp "$classes;lib/*" $main @caseArgs @ResearchArgs
    if ($LASTEXITCODE -ne 0) {throw 'Research checks failed'}
} finally {Pop-Location}
