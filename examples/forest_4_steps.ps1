# forest_4 processed step by step: each option is given at the step that first uses it.
# Usage (from anywhere):
#   .\examples\forest_4_steps.ps1 -Images D:\photos\forest_4 -Project .\projects\forest_4
# Gives the same result as one photogrammetry call with all these options.
param([string]$Images = ".\images", [string]$Project = ".\projects\forest_4")
$ErrorActionPreference = 'Stop'
$P = Join-Path $PSScriptRoot "..\photogrammetry.bat"
$OUT = $Project

function Step([string[]]$arguments) {
  & $P @arguments
  if ($LASTEXITCODE -ne 0) { throw "photogrammetry $($arguments[0]) failed ($LASTEXITCODE)" }
}

# 1. Inputs: images, project, machine resources and resolutions (saved for all later steps).
Step @('inputs',
  '-i', $Images,
  '-o', $OUT,
  '--cores', '12',
  '--memory-mb', '16000',
  '--gpu-memory-mb', '8000',
  '--quality', 'high',
  '--alignment-max-dimension', '1600')

# 2. Calibration. The automatic focal search already detects and matches features, so the
#    feature/matching options belong here; self-calibration also uses the bundle iterations.
Step @('calibrate', '-o', $OUT,
  '--features', '40000',
  '--ratio', '0.75',
  '--neighbors', '3',
  '--descriptor-matcher', 'flann',
  '--descriptor-memory-mb', '1600',
  '--bundle-iterations', '10')

# 3. Matching: pair selection by forward overlap.
Step @('match', '-o', $OUT,
  '--min-forward-overlap', '50')

# 4. Alignment and 5. georeferencing (defaults; add e.g. --gps-sigma 1 to georeference).
Step @('align', '-o', $OUT)
Step @('georeference', '-o', $OUT)

# 6. Dense matching and fusion.
Step @('dense', '-o', $OUT,
  '--step', '1',
  '--filtering', 'moderate',
  '--dense-method', 'sgm',
  '--matching-cost', 'census',
  '--similarity-window', '5',
  '--pyramid-levels', '1',
  '--min-views', '3',
  '--confidence-threshold', '0',
  '--two-view-confidence-threshold', '1.0',
  '--surface-consolidation', 'none',
  '--windowed-mode', 'auto',
  '--fusion-gpu-evidence', 'auto',
  '--fusion-mode', 'image',
  '--fusion-outliers', 'reference',
  '--subpixel-fit', 'equiangular',
  '--pairwise-search', 'tsgm',
  '--stereo-pipeline', 'on',
  '--fusion-strategy', 'graph-average')

# 7. Ground classification, pair correction and ground-aware fusion.
Step @('ground', '-o', $OUT,
  '--classify-ground',
  '--ground-pair-correction',
  '--ground-aware-fusion')

# 8. Report and final project files.
Step @('finish', '-o', $OUT)

& $P status -o $OUT
