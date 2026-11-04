$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Force lib, target/classes, target/test-classes | Out-Null
    $dependency = 'lib/opencv-4.9.0-0.jar'
    if (-not (Test-Path -LiteralPath $dependency)) {
        Invoke-WebRequest 'https://repo.maven.apache.org/maven2/org/openpnp/opencv/4.9.0-0/opencv-4.9.0-0.jar' -OutFile "$dependency.part"
        Move-Item -LiteralPath "$dependency.part" -Destination $dependency
    }
    foreach ($artifact in @('proj4j', 'proj4j-epsg')) {
        $jar = "lib/$artifact-1.4.3.jar"
        if (-not (Test-Path -LiteralPath $jar)) {
            Invoke-WebRequest "https://repo.maven.apache.org/maven2/org/locationtech/proj4j/$artifact/1.4.3/$artifact-1.4.3.jar" -OutFile "$jar.part"
            Move-Item -LiteralPath "$jar.part" -Destination $jar
        }
    }
    foreach ($group in @('imageio','common')) {
        $artifacts = if ($group -eq 'imageio') {@('imageio-tiff','imageio-core','imageio-metadata')} else {@('common-lang','common-io','common-image')}
        foreach ($artifact in $artifacts) {
            $jar = "lib/$artifact-3.15.3.jar"
            if (-not (Test-Path -LiteralPath $jar)) {
                Invoke-WebRequest "https://repo.maven.apache.org/maven2/com/twelvemonkeys/$group/$artifact/3.15.3/$artifact-3.15.3.jar" -OutFile "$jar.part"
                Move-Item -LiteralPath "$jar.part" -Destination $jar
            }
        }
    }
    foreach ($artifact in @('pdfbox','pdfbox-io','fontbox')) {
        $jar = "lib/$artifact-3.0.8.jar"
        if (-not (Test-Path -LiteralPath $jar)) {
            Invoke-WebRequest "https://repo.maven.apache.org/maven2/org/apache/pdfbox/$artifact/3.0.8/$artifact-3.0.8.jar" -OutFile "$jar.part"
            Move-Item -LiteralPath "$jar.part" -Destination $jar
        }
    }
    if (-not (Test-Path -LiteralPath 'lib/commons-logging-1.4.0.jar')) {
        Invoke-WebRequest 'https://repo.maven.apache.org/maven2/commons-logging/commons-logging/1.4.0/commons-logging-1.4.0.jar' -OutFile 'lib/commons-logging-1.4.0.jar.part'
        Move-Item -LiteralPath 'lib/commons-logging-1.4.0.jar.part' -Destination 'lib/commons-logging-1.4.0.jar'
    }
    $dependency = 'lib/*'
    $javaCompiler = 'javac'
    $javaRuntime = 'java'
    if ($env:JAVA_HOME) {
        $javaCompiler = Join-Path $env:JAVA_HOME 'bin/javac.exe'
        $javaRuntime = Join-Path $env:JAVA_HOME 'bin/java.exe'
    } else {
        $portableJdk = Get-ChildItem tools -Directory -Filter 'jdk-25*' -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($portableJdk) {
            $javaCompiler = Join-Path $portableJdk.FullName 'bin/javac.exe'
            $javaRuntime = Join-Path $portableJdk.FullName 'bin/java.exe'
        }
    }
    $sources = @(Get-ChildItem src/main/java -Recurse -Filter '*.java' | ForEach-Object { $_.FullName })
    & $javaCompiler --release 25 -encoding UTF-8 -cp $dependency -d target/classes $sources
    if ($LASTEXITCODE -ne 0) { throw 'Java compilation failed' }
    # Resources (report logo) next to the classes.
    Copy-Item -Recurse -Force src/main/resources/* target/classes/
    $tests =@(Get-ChildItem src/test/java -Recurse -Filter '*.java' | ForEach-Object { $_.FullName })
    & $javaCompiler --release 25 -encoding UTF-8 -cp "target/classes;$dependency" -d target/test-classes $tests
    if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed' }
    & $javaRuntime --enable-native-access=ALL-UNNAMED -ea -cp "target/classes;target/test-classes;$dependency" sgm.SelfTest
    if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
} finally { Pop-Location }
