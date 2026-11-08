package sgm;

import java.io.*;
import java.util.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

/** Rectifies only a requested region; no full-resolution image or rectified raster is allocated. */
final class AerialRectifier implements AerialTileStereo.Rectifier {
  private final DenseReconstruction.Rectified geometry;
  private final Options options;

  AerialRectifier(DenseReconstruction.Rectified geometry, Options options) {
    this.geometry = geometry;
    this.options = options;
  }

  public AerialTileStereo.Pixels read(boolean right, double x, double y, int w, int h, int scale)
      throws IOException {
    var g = geometry;
    ImageFrame frame = right ? g.b : g.a;
    int iw = Math.ceilDiv(frame.width, scale), ih = Math.ceilDiv(frame.height, scale);
    double sx = iw / (double)frame.width, sy = ih / (double)frame.height;
    Camera camera = frame.camera;
    Mat k = Math3.mat(new double[] {camera.fx*sx,0,(camera.cx+.5)*sx-.5,
        0,camera.fy*sy,(camera.cy+.5)*sy-.5,0,0,1},3,3);
    Mat d = new MatOfDouble(frame.calibration.distortion);
    Mat r = Math3.mat(right ? g.r2 : g.r1,3,3);
    double[] projection = (right ? g.p2 : g.p1).clone();
    for (int i=0;i<8;i++) projection[i] /= scale;
    projection[2] -= (g.vertical ? y : x) / scale;
    projection[6] -= (g.vertical ? x : y) / scale;
    Mat p = Math3.mat(projection,3,4), mx = new Mat(), my = new Mat();
    Mat src = new Mat(), srcMask = new Mat(), dst = new Mat(), gray = new Mat(), valid = new Mat();
    try {
      Calib3d.initUndistortRectifyMap(k,d,r,p,
          new Size(g.vertical ? h : w,g.vertical ? w : h),CvType.CV_32FC1,mx,my);
      if (g.vertical) { Core.transpose(mx,mx); Core.transpose(my,my); }
      float[] xs = new float[w*h], ys = new float[w*h];
      mx.get(0,0,xs); my.get(0,0,ys);
      int x0=iw,y0=ih,x1=-1,y1=-1;
      for (int i=0;i<xs.length;i++) {
        if (!Float.isFinite(xs[i]+ys[i]) || xs[i]<-2 || ys[i]<-2
            || xs[i]>=iw+1 || ys[i]>=ih+1) continue;
        x0=Math.min(x0,Math.max(0,(int)Math.floor(xs[i])-1));
        y0=Math.min(y0,Math.max(0,(int)Math.floor(ys[i])-1));
        x1=Math.max(x1,Math.min(iw-1,(int)Math.ceil(xs[i])+1));
        y1=Math.max(y1,Math.min(ih-1,(int)Math.ceil(ys[i])+1));
      }
      if (x1<x0 || y1<y0) return new AerialTileStereo.Pixels(w,h,new byte[w*h],new byte[3*w*h],new byte[w*h]);
      int sw=x1-x0+1,sh=y1-y0+1;
      // Area resampling holds three double sums per source-ROI output pixel.
      if (27L*sw*sh + 32L*w*h > options.memoryMb*1048576L/4)
        throw new AerialTileStereo.TileTooLarge("Rectified source footprint exceeds tile host budget");
      byte[] bgr;
      try (var reader = new WindowedImage.RegionReader(frame,iw,ih)) {
        bgr=scale>1 ? reader.preview(x0,y0,sw,sh) : reader.read(x0,y0,sw,sh);
      }
      for(int i=0;i<xs.length;i++) { xs[i]-=x0; ys[i]-=y0; }
      mx.put(0,0,xs); my.put(0,0,ys);
      src.create(sh,sw,CvType.CV_8UC3); src.put(0,0,bgr);
      srcMask.create(sh,sw,CvType.CV_8U); srcMask.setTo(new Scalar(255));
      Imgproc.remap(src,dst,mx,my,Imgproc.INTER_LINEAR,Core.BORDER_CONSTANT,new Scalar(0));
      Imgproc.remap(srcMask,valid,mx,my,Imgproc.INTER_LINEAR,Core.BORDER_CONSTANT,new Scalar(0));
      Imgproc.threshold(valid,valid,254,255,Imgproc.THRESH_BINARY);
      Mat kernel=Mat.ones(Math.max(options.census,options.correlationWindow)+2,
          Math.max(options.census,options.correlationWindow)+2,CvType.CV_8U);
      try { Imgproc.erode(valid,valid,kernel,new Point(-1,-1),1,Core.BORDER_CONSTANT,new Scalar(0)); }
      finally { kernel.release(); }
      Imgproc.cvtColor(dst,gray,Imgproc.COLOR_BGR2GRAY);
      byte[] colors=new byte[3*w*h],gdata=new byte[w*h],mask=new byte[w*h];
      dst.get(0,0,colors);gray.get(0,0,gdata);valid.get(0,0,mask);
      return new AerialTileStereo.Pixels(w,h,gdata,colors,mask);
    } finally {
      for(Mat m:new Mat[]{k,d,r,p,mx,my,src,srcMask,dst,gray,valid})m.release();
    }
  }
}
