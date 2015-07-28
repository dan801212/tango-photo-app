# tango-photo-app
This app is for helping blind or visual-impaired people to aim properly while they take photos. The main architecture is based on example provided by Google.

1. Develope Environment:
  
  To run the program, you need to download Android Studio as well as Android NDK. Then modify the NDK path inside local.properties file to you own directory.

2. Project intro.:

  By using NDK, this project combined with Java and Cpp part.
  Function regarding to Android are mainly in Java, such as UI, thread, or activity.
  While C++ part contains Tango API including data retrieving, processing, and rendering(by Opengl-ES).
  
  In current version, data processing was implemented including: 
  1. RANSAC, which eliminate the largest plane in view.
  2. Filter, which remove the point that is too far from the closest point (in Z-axis).
  
  You can find these code in tango_data.cpp (mainly in function onXYZijAvailable())


p.s Tango API should be updated in case of you want to use some latest function.

Please contact dan801212@gmail.com if you got further question.
