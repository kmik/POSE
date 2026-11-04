# Third-party dependencies

POSE has no project license assigned yet. This does not alter the licenses of its dependencies. Runtime jars are downloaded by the build and are not committed to this source distribution.

| Component | Pinned artifact | Upstream notices |
| --- | --- | --- |
| OpenPnP OpenCV Java/native distribution | `org.openpnp:opencv:4.9.0-0` | [Packaging project](https://github.com/openpnp/opencv), [OpenCV 4.9 license](https://github.com/opencv/opencv/blob/4.9.0/LICENSE) |
| Proj4J coordinate transformations | `org.locationtech.proj4j:proj4j:1.4.3` | [Proj4J license](https://github.com/locationtech/proj4j/blob/master/LICENSE) |
| Proj4J EPSG definitions | `org.locationtech.proj4j:proj4j-epsg:1.4.3` | [EPSG database license](https://github.com/locationtech/proj4j/blob/master/LICENSE.EPSG), plus the upstream project license |
| TwelveMonkeys TIFF/BigTIFF ImageIO reader and common/core/metadata modules | `com.twelvemonkeys.imageio:imageio-tiff:3.15.3` and its runtime dependencies | [Upstream BSD notices](https://github.com/haraldk/TwelveMonkeys#license) |
| Apache PDFBox (PDF report), with FontBox and PDFBox I/O | `org.apache.pdfbox:pdfbox:3.0.8` | [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0), [PDFBox notices](https://github.com/apache/pdfbox/blob/3.0.8/NOTICE.txt) |
| Apache Commons Logging (PDFBox dependency) | `commons-logging:commons-logging:1.4.0` | [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) |

The OpenPnP artifact's embedded Maven metadata declares a BSD License; OpenCV 4.9's upstream source includes Apache 2.0 terms and third-party notices. Consult the packaged/upstream notices for the components actually redistributed. Proj4J EPSG metadata declares Apache 2.0 and the EPSG database distribution license. Preserve applicable notices when redistributing runtime binaries; this repository does not replace them with a project-wide grant.

LAS output uses the published [ASPRS LAS specification](https://www.asprs.org/wp-content/uploads/2019/07/LAS_1_4_r15.pdf). Algorithm/background references remain in the technical documentation. Local JDKs, formatting tools, original C++ third-party libraries, downloaded reference software and generated datasets are excluded from the standalone source package.

The example point-cloud image has no separate license.
