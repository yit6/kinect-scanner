
// Daniel Shiffman
// Kinect Point Cloud example

// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/

import org.openkinect.freenect.*;
import org.openkinect.processing.*;

// Kinect Library object
Kinect kinect;

// Angle for rotation
float a = 0;

boolean saved = true;
int fileNum = 0;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

void setup() {
  // Rendering in P3D
  size(800, 600, P3D);
  kinect = new Kinect(this);
  kinect.initDepth();

  println(kinect.getTilt());

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
}

void draw() {

  background(0);

  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();

  // Number of points to skip when displaying the image
  int skip = 4;

  // Center the image
  translate(width/2, height/2, -50);

  // Make ArrayLists to store all to vertices and faces
  ArrayList<Vertex> vertices = new ArrayList();
  ArrayList<Face> faces = new ArrayList();

  // Nested for loop that initializes x and y pixels and, for those less than the
  // maximum threshold and at every skiping point, the offset is caculated to map
  // them on a plane instead of just a line
  for (int x = 0; x < kinect.width; x += skip) {
    for (int y = 0; y < kinect.height; y += skip) {
      int offset = x + y*kinect.width;

      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);


      if (v.z < 1.5) {
        stroke(255);
      } else {
        stroke(100);
      }
      point(map(v.x, 0, 3, 0, width), map(v.y, 0, 3, 0, height));
    }
  }

  // If the current sensor data should be saved
  if (!saved) {

    // Add all the points to the vertices ArrayList
    for (int y = 0; y < kinect.height; y++) {
      for (int x = 0; x < kinect.width; x++) {
        int offset = x + y*kinect.width;

        // Convert kinect data to world xyz coordinate
        int rawDepth = depth[offset];
        PVector v = depthToWorld(x, y, rawDepth);
        Vertex vertex = new Vertex(v.x, v.y, v.z);

        // If the point is too far away, or has no depth value, it should be deleted later
        if (v.z < 1.5 && v.z != 0) {
          vertex.ignore = false;
        }
        vertices.add(vertex);
      }
    }

    // Add all the faces to the faces ArrayList
    for (int x = 0; x < kinect.width-1; x++) {
      for (int y = 0; y < kinect.height-1; y++) {

        // Only add the face if all the vertices in the face are valid
        if (
          !vertices.get( y      * kinect.width + x    ).ignore &&
          !vertices.get((y + 1) * kinect.width + x    ).ignore &&
          !vertices.get( y      * kinect.width + x + 1).ignore &&
          !vertices.get((y + 1) * kinect.width + x + 1).ignore
          ) {
          faces.add(new Face(
            vertices.get( y      * kinect.width + x    ), 
            vertices.get((y + 1) * kinect.width + x    ), 
            vertices.get( y      * kinect.width + x + 1), 
            vertices.get((y + 1) * kinect.width + x + 1)
            ));
        }
      }
    }

    // Remove all the invalid vertices
    for (int i = vertices.size()-1; i >= 0; i--) {
      if (vertices.get(i).ignore) {
        vertices.remove(i);
      }
    }

    // Recalculate the indices of the vertices, used by the faces
    for (int i = 0; i < vertices.size(); i++) {
      vertices.get(i).index = i;
    }

    // Make a string array to save as an obj file
    String[] obj = new String[vertices.size() + faces.size()];

    // Add all the vertices to the file
    for (int i = 0; i < vertices.size(); i++) {
      obj[i] = vertices.get(i).toString();
    }

    // Add all the faces to the file
    for (int i = 0; i < faces.size(); i++) {
      obj[i+vertices.size()] = faces.get(i).toString();
    }

    // Save the string array
    saveStrings("scan"+fileNum+".obj", obj);
    fileNum++;
    saved = true;
  }

  // Rotate
  a += 0.015f;
}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

// Only needed to make sense of the ouput depth values from the kinect
PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  // Drawing the result vector to give each point its three-dimensional space
  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

void mousePressed() {
  saved = false;
}

class Vertex {
  float x, y, z;
  int index;
  boolean ignore = true;

  Vertex(float x, float y, float z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  String toString() {
    return "v " + x + " " + y + " " + z;
  }
}

class Face {
  Vertex a, b, c, d;

  Face(Vertex a, Vertex b, Vertex c, Vertex d) {
    this.a = a;
    this.b = b;
    this.c = c;
    this.d = d;
  }

  String toString() {
    return "f " + (a.index+1)
          + " " + (b.index+1)
          + " " + (d.index+1)
          + " " + (c.index+1);
  }
}
