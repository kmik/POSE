# Interior and exterior orientation exchange

Use `--orientation-input FILE_OR_DIRECTORY` for a combined source, or `--calibration` and `--poses` separately. Detection is automatic; `--orientation-input-format`, `--calibration-format` and `--poses-format` select an adapter. Filenames associate cameras with photographs; duplicate basenames fail. Distinct per-image calibrations require `--intrinsics-mode fixed`. Dense pairs still require equal processed dimensions.

Examples from `sgm_java` (on Linux replace `.\photogrammetry.bat` with `bash photogrammetry.sh`):

```powershell
.\photogrammetry.bat -i photos -o project --orientation-input colmap/sparse/0 --intrinsics-mode fixed --extrinsics-mode fixed
.\photogrammetry.bat -i photos -o project --calibration calibration.yml --calibration-format opencv --poses exteriors.csv --poses-format csv-opk
.\photogrammetry.bat -i photos -o project --orientation-output-format csv,colmap,metashape
.\photogrammetry.bat --export-orientations output/my-project/runs/RUN_ID --orientation-output-format csv,colmap,metashape --orientation-output camera-export
```

Supplied poses/calibration are fixed by default. `--extrinsics-mode prior|initial` and `--intrinsics-mode prior|initial` enable refinement, with uncertainty controls described in help.

## Export after processing

`--export-orientations RUN_DIRECTORY` uses stored cameras without matching, depth computation or original photographs. New runs retain source and processed camera models in `04_geometry/orientation_model.json`. Older runs recover from the image inventory, reference calibration, processed camera table and final poses. Choose a **run**, not the parent project directory.

`--orientation-output-format LIST` exports combined cameras. `--interior-output-format LIST` and `--exterior-output-format LIST` add formats independently. CSV creates `interiors.csv`, `exteriors.csv` and `orientations.csv`; formats with combined camera containers write both orientations together. Defaults: no export during reconstruction, CSV for standalone export. Each format gets a directory and `conventions.json`, with a top-level manifest. Existing format directories are preserved; choose a new destination to export again. Conversions are staged before publication.

## Coordinates, units and pixels

Internally: `X_camera=R*X_world+t`, center `C=-R^T*t`, camera axes **x right, y down, z forward**, upper-left pixel **center** `(0,0)`. Intrinsics are in pixels; distortion order is OpenCV `k1,k2,p1,p2,k3`. Invalid rotations, non-finite parameters, unsupported nonzero skew and unsupported projections fail explicitly.

Input cameras must describe the stored, unrotated decoded raster: the image loader ignores EXIF display orientation. If an external workflow rotated/cropped photographs, apply the same image-coordinate convention before import; do not combine its calibration with differently oriented raw pixels.

`--orientation-pixels source` (default) exports original distorted raster intrinsics with final poses. `processed` exports dense undistorted raster intrinsics and zero distortion. Processed calibration belongs with processed photographs, not raw originals. Resize mapping is `cx'=(cx+0.5)*sx-0.5`, similarly for y; focal lengths scale by sx/sy.

`--orientation-crs EPSG:CODE` declares already-projected metre coordinates; it does **not** transform them. It suppresses GPS realignment, conflicts with `--gps`, and requires `--scale 1`. `--orientation-world-units m|ft|mm` declares units without rescaling; projected processing requires metres. CRS/units are retained in sidecars; receiving software may require separate CRS setup. No vertical-datum conversion occurs.

## Supported variants

| Name | Input | Output / limits |
|---|---|---|
| `native` | Existing calibration properties/14-number calibration, pose CSV, canonical camera JSON | Complete `orientations.json` with CRS metadata |
| `csv` | Header-based intrinsics; matrix or quaternion exteriors | Combined and separate CSV with Brown distortion |
| `csv-opk` | Camera XYZ centers and omega/phi/kappa | Explicit OPK convention below |
| `colmap` | Text cameras/images, or little-endian cameras.bin/images.bin | Text sparse model, orientations only |
| `bundler` | Bundle v0.3 `.out` and associated image list | One focal, centered principal point, k1/k2; dimensions sidecar included |
| `openmvg` | sfm_data.json pinhole/radial/Brown cameras | JSON 0.3, Brown cameras, empty structure; one focal |
| `opensfm` | Perspective/Brown reconstruction JSON | Brown JSON, empty points; component selected with `--orientation-component` |
| `metashape` | Frame calibration XML or exported cameras XML | Frame sensors/cameras XML; no proprietary project archives |
| `micmac` | Matrix OrientationConique and linked/embedded CalibrationInternConique | Radial ModRad calibration and per-image XML; square pixels |
| `opencv` | OpenCV XML/YAML calibration and optional poses | Multi-camera FileStorage XML |
| `inpho` | Digital pinhole PRJ 5/8 PHOTO/EXT_ORI with camera definitions | Pinhole PRJ; physical pixel pitch required |

These are specific interchange variants, not every proprietary project version. Independent convention fixtures and round trips cover the adapters; the external applications themselves have not all been exercised here. XML external entities are disabled. Unknown models fail instead of silently becoming pinhole cameras.

### CSV conventions

Interior columns: `image,width,height,fx,fy,cx,cy,k1,k2,p1,p2,k3`. `filename` aliases `image`; optional skew must be zero. Exterior matrix header:

```text
image,r00,r01,r02,r10,r11,r12,r20,r21,r22,tx,ty,tz
```

Quaternion input uses `qw,qx,qy,qz,tx,ty,tz`, world-to-camera, normalized on import. OPK uses `image,x,y,z,omega,phi,kappa` (or center_x/center_y/center_z), with camera centers, not translation. The photogrammetric **camera-to-world** matrix is `Rz(kappa)*Ry(phi)*Rx(omega)`, then converted from x-right/y-up/z-backward axes. Angles default to degrees; `--orientation-angle-unit radians` changes CSV import/export. Other vendors' OPK conventions need conversion first. Near phi=90 degrees, Euler export fails explicitly; use matrix/quaternion output.

### Format-specific details

- **COLMAP:** world-to-camera quaternion and t. Corner-origin principal point shifts -0.5 on import and +0.5 on export. Supported camera models: SIMPLE_PINHOLE, PINHOLE, SIMPLE_RADIAL, RADIAL, OPENCV, FULL_OPENCV with zero rational-denominator terms. Fisheye models fail. Text export includes empty points3D.txt; no observations/structure are implied.
- **Bundler:** y-up/backward-z camera axes are converted. Dimensions require input image headers or sibling images.csv; ordered filenames require list.txt or `--orientation-image-list`. This adapter uses width/2,height/2 as centered principal point. Other intrinsic terms need explicit lossy export.
- **OpenMVG:** extrinsic center converts to t. Brown `disto_t2` serializes **five** values k1,k2,k3,t1,t2, reordered to OpenCV; it is not a two-value tangential array.
- **OpenSfM:** normalized focal/principal point scale by max(width,height), centered at `(width-1)/2,(height-1)/2`. Pose Rodrigues vectors are world-to-camera. reference_lla alone does not assign an EPSG CRS.
- **Metashape:** applies camera-to-world transforms and chunk similarity, prefers adjusted frame calibration, converts principal-point offsets/pixel origin, and swaps tangential p1/p2 to OpenCV. Nonzero b2 skew fails; k4/p3/p4 need lossy conversion. **Reference WKT is not automatically assigned to transformed XYZ**: chunk coordinates can be geocentric even with a projected reference CRS. WKT is retained in notes. Transform externally or explicitly declare a verified matching frame.
- **MicMac:** default matrix direction is camera-to-world; `--orientation-rotation world-to-camera` overrides a known alternate. Supports identity image-to-camera affine and forward `eConvApero_DistM2C` radial distortion, converting pixel coefficients by focal^(2*i). A separate distortion center is lossy. Inverse distortion, other projections and nonidentity scan/crop affines require conversion first.
- **OpenCV:** zero-skew K; five Brown terms or additional zero terms. XML accepts camera_matrix/K, distortion_coefficients/distCoeffs/D, dimensions, rotation_matrix+translation_vector or rvec+tvec. YAML accepts corresponding OpenCV-matrix data blocks. Multi-camera XML uses a cameras sequence with image names. Single-camera YAML is primarily a shared calibration source.
- **INPHO:** digital pinhole EXT_ORI defaults to world-to-camera photogrammetric axes (x right, y up, z backward); conversion to OpenCV negates rows 2 and 3 without transposing the matrix. `--orientation-rotation camera-to-world` supports explicitly known alternate files; `--orientation-axes` overrides camera axes. Exports use world-to-camera EXT_ORI. Files exported by the old transposing implementation need the explicit camera-to-world override. CCD dimensions and diagonal CCD_INTERIOR_ORIENTATION map photo millimetres to pixels; active calibration sets are selected by ID. Rotated/skewed/reflected scanner affines fail. Nonzero INPHO polynomial/grid corrections are not claimed equivalent to OpenCV Brown: rejected by default, or explicitly dropped with lossy conversion. `--orientation-pixel-size MM` supplies absent pitch and is required for export. Export needs square pixels and zero distortion, so processed pixels are usually appropriate. Local INPHO export needs declared world units. Analog/scanner and gridded projects need conversion first.

`--orientation-allow-lossy` allows explicitly reported intrinsic approximations; it does not bypass invalid rotations, unsupported projections or missing dimensions. Preserve conventions.json with exchanged files.

Primary references: [COLMAP](https://colmap.github.io/format.html), [Bundler](https://www.cs.cornell.edu/~snavely/bundler/bundler-v0.3-manual.html), [OpenMVG Brown serialization](https://github.com/openMVG/openMVG/blob/develop/src/openMVG/cameras/Camera_Pinhole_Brown_io.hpp), [OpenSfM serialization](https://github.com/mapillary/OpenSfM/blob/main/opensfm/io.py), [Metashape manual](https://www.agisoft.com/pdf/metashape-pro_2_3_en.pdf), [MicMac documentation](https://github.com/micmacIGN/Documentation).
