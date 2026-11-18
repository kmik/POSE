param([switch]$Install)
$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
  $checkOut = 'target/host-fusion-check-' + (Get-Date -Format 'yyyyMMddTHHmmss') + '-' + $PID
  New-Item -ItemType Directory "$checkOut/classes", "$checkOut/tests" | Out-Null
  $checkJavac = 'javac'; $checkJava = 'java'
  if ($env:JAVA_HOME) { $checkJavac = Join-Path $env:JAVA_HOME 'bin/javac.exe'; $checkJava = Join-Path $env:JAVA_HOME 'bin/java.exe' }
  if ($env:POSE_JAVA_HOME) { $checkJavac = Join-Path $env:POSE_JAVA_HOME 'bin/javac.exe'; $checkJava = Join-Path $env:POSE_JAVA_HOME 'bin/java.exe' }
  $mainSources = @(Get-ChildItem src/main/java -Recurse -Filter '*.java' | ForEach-Object FullName)
  & $checkJavac --release 25 -encoding UTF-8 -cp 'lib/*' -d "$checkOut/classes" $mainSources
  if ($LASTEXITCODE -ne 0) { throw 'Engine compilation failed' }
  Copy-Item -Recurse -Force src/main/resources/* "$checkOut/classes/"
  $testSources = @(Get-ChildItem src/test/java -Recurse -Filter '*.java' | ForEach-Object FullName)
  & $checkJavac --release 25 -encoding UTF-8 -cp "$checkOut/classes;lib/*" -d "$checkOut/tests" $testSources
  if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed' }
  $suiteText = [IO.File]::ReadAllText((Join-Path (Get-Location) 'src/test/java/sgm/SelfTest.java'))
  $replacements = @{
    'public final class SelfTest {' = 'public final class HostMemorySuite {'
    'checks += BundleQualityRegression.run(root);' = 'System.out.println("SKIP BundleQualityRegression: high-only validation");'
    'checks += MultiViewDisparityRegression.run(root.resolve("multiview-disparity"));' = 'System.out.println("SKIP MultiViewDisparityRegression: high-only validation");'
  }
  foreach ($entry in $replacements.GetEnumerator()) {
    if ([regex]::Matches($suiteText,[regex]::Escape($entry.Key)).Count -ne 1) { throw "Review high-only harness: $($entry.Key)" }
    $suiteText = $suiteText.Replace($entry.Key, $entry.Value)
  }
  [IO.File]::WriteAllText((Join-Path (Get-Location) "$checkOut/HostMemorySuite.java"), $suiteText, [Text.UTF8Encoding]::new($false))
  & $checkJavac --release 25 -encoding UTF-8 -cp "$checkOut/classes;$checkOut/tests;lib/*" -d "$checkOut/tests" "$checkOut/HostMemorySuite.java"
  if ($LASTEXITCODE -ne 0) { throw 'High-only harness compilation failed' }
  $previousErrorPreference = $ErrorActionPreference
  # Keep native OpenCV artifact paths below Windows MAX_PATH in deeply nested fixtures.
  $checkTestRoot = Join-Path ([IO.Path]::GetTempPath()) 'sgmh'
  try {
    $ErrorActionPreference = 'Continue'
    & $checkJava --enable-native-access=ALL-UNNAMED -Xmx8g -XX:ActiveProcessorCount=4 "-Dsgm.test.root=$checkTestRoot" -ea -cp "$checkOut/classes;$checkOut/tests;lib/*" sgm.HostMemorySuite 2>&1 | Tee-Object "$checkOut/checks.log"
  } finally { $ErrorActionPreference = $previousErrorPreference }
  if ($LASTEXITCODE -ne 0) { throw "High-only regression failed; see $checkOut/checks.log" }
  if ($Install) { New-Item -ItemType Directory -Force target/classes | Out-Null; Copy-Item -Recurse -Force "$checkOut/classes/*" target/classes/ }
  Write-Output "High-only validation artifacts: $checkOut (two ultra-fixture suites skipped)."
} finally { Pop-Location }
