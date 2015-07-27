/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define GLM_FORCE_RADIANS

#include <jni.h>
#include <string>
#include <stdlib.h>
#include <dirent.h>
#include <fstream>

#include "tango-gl/transform.h"
#include "tango-gl/axis.h"
#include "tango-gl/camera.h"
#include "tango-gl/color.h"
#include "tango-gl/conversions.h"
#include "tango-gl/cube.h"
#include "tango-gl/frustum.h"
#include "tango-gl/grid.h"
#include "tango-gl/goal_marker.h"
#include "tango-gl/trace.h"
#include "tango-gl/util.h"
#include "tango-gl/video_overlay.h"

#include "pointcloud.h"
#include "tango_data.h"

// Screen size.
GLuint screen_width;
GLuint screen_height;

// Render camera's parent transformation.
// This object is a pivot transformtion for render camera to rotate around.
tango_gl::Transform* cam_parent_transform;

// Render camera.
tango_gl::Camera* cam;
tango_gl::Camera* cam2;

// Device frustum.
tango_gl::Frustum* frustum;

// Point cloud drawable object.
Pointcloud* pointcloud;

// Device axis (in OpenGL coordinates).
tango_gl::Axis* axis;

// Ground grid.
// Each block is 1 meter by 1 meter in real world.
tango_gl::Grid* grid;

//6.24
// Color camera preview.
tango_gl::VideoOverlay* video_overlay;

GLuint small_screen_width;
GLuint small_screen_height;
GLuint temp_width;
GLuint temp_height;
// Single finger touch positional values.
// First element in the array is x-axis touching position.
// Second element in the array is y-axis touching position.
float cam_start_angle[2];
float cam_cur_angle[2];

// Double finger touch distance value.
float cam_start_dist;
float cam_cur_dist;

enum CameraType {
  FIRST_PERSON = 0,
  THIRD_PERSON = 1,
  TOP_DOWN = 2
};
CameraType camera_type;
//6.25
unsigned char *pixel = NULL;
unsigned char *topdown_pixel = NULL;
GLuint fbuffer, rbuffers[2];
bool saveNextFrame = false;

// Render and camera controlling constant values.
// Height offset is used for offset height of motion tracking
// pose data. Motion tracking start position is (0,0,0). Adding
// a height offset will give a more reasonable pose while a common
// human is holding the device. The units is in meters.
const glm::vec3 kHeightOffset = glm::vec3(0.0f, 1.3f, 0.0f);

// Render camera observation distance in third person camera mode.
const float kThirdPersonCameraDist = 7.0f;

// Render camera observation distance in top down camera mode.
const float kTopDownCameraDist = 5.0f;

// Zoom in speed.
const float kZoomSpeed = 10.0f;

// Min/max clamp value of camera observation distance.
const float kCamViewMinDist = 1.0f;
const float kCamViewMaxDist = 100.f;

// FOV set up values.
// Third and top down camera's FOV is 65 degrees.
// First person camera's FOV is 37.8 degrees.
// The first person FOV is set to similar FOV of physical camera.
const float kHighFov = 65.0f;
const float kLowFov = 37.8f;

// Frustum scale.
const glm::vec3 kFrustumScale = glm::vec3(0.4f, 0.3f, 0.5f);

// Color of the ground grid.
const tango_gl::Color kGridColor(0.85f, 0.85f, 0.85f);

// Set camera type, set render camera's parent position and rotation.
void SetCamera(CameraType camera_index) {
  camera_type = camera_index;
  cam_cur_angle[0] = cam_cur_angle[1] = cam_cur_dist = 0.0f;
  switch (camera_index) {
    case CameraType::FIRST_PERSON:
      cam->SetFieldOfView(kLowFov);
      cam_parent_transform->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
      cam_parent_transform->SetRotation(glm::quat(1.0f, 0.0f, 0.0, 0.0f));
      break;
    case CameraType::THIRD_PERSON:
      cam->SetFieldOfView(kHighFov);
      cam->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
      cam->SetRotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
      cam_cur_dist = kThirdPersonCameraDist;
      cam_cur_angle[0] = -M_PI / 4.0f;
      cam_cur_angle[1] = M_PI / 4.0f;
      break;
    case CameraType::TOP_DOWN:
      cam->SetFieldOfView(kHighFov);
      cam->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
      cam->SetRotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
      cam_cur_dist = kTopDownCameraDist;
      cam_cur_angle[1] = M_PI / 2.0f;
      break;
    default:
      break;
  }
}

bool InitGlContent() {
  camera_type = CameraType::FIRST_PERSON;

  cam = new tango_gl::Camera();

  pointcloud = new Pointcloud();
  frustum = new tango_gl::Frustum();
  axis = new tango_gl::Axis();
  grid = new tango_gl::Grid();
  cam_parent_transform = new tango_gl::Transform();

  frustum->SetScale(kFrustumScale);

  // Set the parent-child camera transfromation.
  cam->SetParent(cam_parent_transform);

  // Put the grid at the resonable height since the motion
  // tracking pose always starts at (0, 0, 0).
  grid->SetPosition(kHeightOffset);
  grid->SetColor(kGridColor);
  SetCamera(CameraType::FIRST_PERSON);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  return true;
}

bool SetupGraphics(int w, int h) {
  screen_width = w;
  screen_height = h;

  if (h == 0) {
    LOGE("Setup graphic height not valid");
    return false;
  }
  cam->SetAspectRatio(static_cast<float>(w) / static_cast<float>(h));
  return true;
}

// GL render loop.
bool RenderFrame() {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  // XYZij dirty indicates that the XYZij data has been changed.
  if (TangoData::GetInstance().is_xyzij_dirty) {
    TangoData::GetInstance().UpdateXYZijData();
  }

  // Pose dirty indicates that the pose data has been changed.
  if (TangoData::GetInstance().is_pose_dirty) {
    TangoData::GetInstance().UpdatePoseData();
  }

  /// Viewport set to full screen, and camera at origin
  /// facing on negative z direction, y axis is the up
  /// vector of the camera.
  glViewport(0, 0, screen_width, screen_height);

  // Get OpenGL camera to OpenGL world transformation for motion tracking.
  // Computed based on pose callback.
  glm::mat4 oc_2_ow_mat_motion = glm::mat4(1.0f);

  // Get OpenGL camera to OpenGL world transformation for depth.
  // Note that this transformation is different from the oc_2_ow_mat_motion
  // due to the timestamp differences. This transformation is computed
  // based on the closest pose data of depth frame timestamp.
  glm::mat4 oc_2_ow_mat_depth = glm::mat4(1.0f);

  if (camera_type == CameraType::FIRST_PERSON) {
    // Get motion transformation.
    oc_2_ow_mat_motion = TangoData::GetInstance().GetOC2OWMat(false);

    // Set camera's pose to motion tracking's pose.
    cam->SetTransformationMatrix(oc_2_ow_mat_motion);

    // Get depth frame transformation.
    oc_2_ow_mat_depth = TangoData::GetInstance().GetOC2OWMat(true);
  } else {
    // Get parent camera's rotation from touch.
    // Note that the render camera is a child transformation
    // of the this transformation.
    // cam_cur_angle[0] is the x-axis touch, cooresponding to y-axis rotation.
    // cam_cur_angle[0] is the y-axis touch, cooresponding to x-axis rotation.
    glm::quat parent_cam_rot =
        glm::rotate(glm::quat(1.0f, 0.0f, 0.0f, 0.0f), -cam_cur_angle[0],
                    glm::vec3(0, 1, 0));
    parent_cam_rot =
        glm::rotate(parent_cam_rot, -cam_cur_angle[1], glm::vec3(1, 0, 0));

    // Get motion transformation.
    oc_2_ow_mat_motion = TangoData::GetInstance().GetOC2OWMat(false);

    // Get depth frame transformation.
    oc_2_ow_mat_depth = TangoData::GetInstance().GetOC2OWMat(true);

    // Set render camera parent position and rotation.
    cam_parent_transform->SetRotation(parent_cam_rot);
    cam_parent_transform->SetPosition(
        tango_gl::util::GetTranslationFromMatrix(oc_2_ow_mat_motion));

    frustum->SetTransformationMatrix(oc_2_ow_mat_motion);
    frustum->SetScale(kFrustumScale);
    frustum->Render(cam->GetProjectionMatrix(), cam->GetViewMatrix());

    // Set camera view distance, based on touch interaction.
    cam->SetPosition(glm::vec3(0.0f, 0.0f, cam_cur_dist));
  }

  // Set axis transformation, axis representing device's pose.
  axis->SetTransformationMatrix(oc_2_ow_mat_motion);
  axis->Render(cam->GetProjectionMatrix(), cam->GetViewMatrix());

  // Render point cloud based on depth buffer and depth frame transformation.
  pointcloud->Render(
      cam->GetProjectionMatrix(), cam->GetViewMatrix(), oc_2_ow_mat_depth,
      TangoData::GetInstance().depth_buffer_size * 3,
      static_cast<float*>(TangoData::GetInstance().depth_buffer));

  grid->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f) - kHeightOffset);
  // Render grid.
  grid->Render(cam->GetProjectionMatrix(), cam->GetViewMatrix());
  return true;
}

bool CameraInitGlContent() {
  cam2 = new tango_gl::Camera();
  video_overlay = new tango_gl::VideoOverlay();
  cam2->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
  cam2->SetRotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  return true;
}
bool CameraSetupGraphics(int w, int h) {
  small_screen_width = w;
  small_screen_height = h;
  temp_height = h;
  temp_width = w;

  if (h == 0) {
    LOGE("Setup graphic height not valid");
    return false;
  }
  cam2->SetAspectRatio(static_cast<float>(w) / static_cast<float>(h));
  glClearColor(0.3f, 0.3f, 0.3f, 1.0f);

  if(pixel != NULL){
  free(pixel);
  free(topdown_pixel);
  }
       pixel = (unsigned char *) calloc(w * h * 3, sizeof(unsigned char)); //3 components
       topdown_pixel = (unsigned char *) calloc(w * h * 3, sizeof(unsigned char));
       if(pixel == NULL)  {
                    LOGE("memory not allocated!");
                         return false;
                         }

      // Create and bind the framebuffer
      glGenFramebuffers(1, &fbuffer);
      glBindFramebuffer(GL_FRAMEBUFFER, fbuffer);

      glGenRenderbuffers(2, rbuffers);
      glBindRenderbuffer(GL_RENDERBUFFER, rbuffers[0]);

      glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB565, w, h);
      glBindRenderbuffer(GL_RENDERBUFFER, rbuffers[1]);
      glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, w, h);

      glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rbuffers[0]);
      glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,  GL_RENDERBUFFER, rbuffers[1]);

      if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
                                                      LOGE("Framebuffer not complete");
                                                      return false;
      }
      glBindFramebuffer(GL_FRAMEBUFFER, 0);


  return true;
}
bool CameraRenderFrame() {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  /// Viewport set to full screen, and camera at origin
  /// facing on negative z direction, y axis is the up
  /// vector of the camera.
  glViewport(0, 0, small_screen_width, small_screen_height);

  // UpdateColorTexture() updates color camera's
  // texture and timestamp.
  TangoData::GetInstance().UpdateColorTexture();
  video_overlay->CameraRender(glm::mat4(1.0f), glm::mat4(1.0f));

  //6.25
  if(saveNextFrame){

  //change to smaller screen size
  small_screen_width = 600;
  small_screen_height = 400;

    if (small_screen_height == 0) {
      LOGE("Setup graphic height not valid");
      return false;
    }
    cam2->SetAspectRatio(static_cast<float>(small_screen_width) / static_cast<float>(small_screen_height));
    glClearColor(0.3f, 0.3f, 0.3f, 1.0f);

    if(pixel != NULL){
    free(pixel);
    free(topdown_pixel);
    }
         pixel = (unsigned char *) calloc(small_screen_width * small_screen_height * 3, sizeof(unsigned char)); //3 components
         topdown_pixel = (unsigned char *) calloc(small_screen_width * small_screen_height * 3, sizeof(unsigned char));
         if(pixel == NULL)  {
                      LOGE("memory not allocated!");
                           return false;
                           }

        // Create and bind the framebuffer
        glGenFramebuffers(1, &fbuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, fbuffer);

        glGenRenderbuffers(2, rbuffers);
        glBindRenderbuffer(GL_RENDERBUFFER, rbuffers[0]);

        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB565, small_screen_width, small_screen_height);
        glBindRenderbuffer(GL_RENDERBUFFER, rbuffers[1]);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, small_screen_width, small_screen_height);

        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rbuffers[0]);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,  GL_RENDERBUFFER, rbuffers[1]);

        if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
                                                        LOGE("Framebuffer not complete");
                                                        return false;
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
      glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

      /// Viewport set to full screen, and camera at origin
      /// facing on negative z direction, y axis is the up
      /// vector of the camera.
      glViewport(0, 0, small_screen_width, small_screen_height);

      // UpdateColorTexture() updates color camera's
      // texture and timestamp.
      TangoData::GetInstance().UpdateColorTexture();
      video_overlay->CameraRender(glm::mat4(1.0f), glm::mat4(1.0f));

  // save the output of glReadPixels somewhere... I'm a noob about JNI however, so I leave this part as an exercise for the reader ;-)
  glReadPixels(0, 0, small_screen_width, small_screen_height, GL_RGB, GL_UNSIGNED_BYTE, pixel);
  int end = small_screen_width * small_screen_height * 3 - 1;
//   This function returns the same results regardless of where the data comes from (screen or fbo)
  LOGI("glReadPixel => (%hhu,%hhu,%hhu), (%hhu,%hhu,%hhu), (%hhu,%hhu,%hhu), (%hhu,%hhu,%hhu),...,(%hhu, %hhu, %hhu)",
        pixel[0], pixel[1], pixel[2], pixel[3],
        pixel[4], pixel[5], pixel[6], pixel[7],
        pixel[8], pixel[9], pixel[10], pixel[11],
        pixel[end - 2], pixel[end - 1], pixel[end]);

  for( int j=0; j<small_screen_height; j++ ){
      memcpy( &topdown_pixel[j*small_screen_width*3], &pixel[(small_screen_height-j-1)*small_screen_width*3], small_screen_width*3*sizeof(unsigned char) );
  }
  DIR* dir = opendir("/storage/emulated/0/Pictures/TangoCaptures/");
  struct dirent* ent;
  size_t extLocate;

  if(dir != NULL){
    while((ent = readdir(dir)) != NULL){
//      extLocate = std::string(ent->d_name).find_last_of(".");
//      std::string n = std::string(ent->d_name).substr(extLocate-4, extLocate);
    }
      double index = TangoData::GetInstance().timestamp;
      std::ostringstream strs;
      strs << "/storage/emulated/0/Pictures/TangoCaptures/pic" << index << ".ppm";
      std::string stname = strs.str();

      FILE *f0 = fopen( stname.c_str() , "wb" );
      if( f0==NULL )
      {
        //printf( "[Error] : SaveScreen(), Cannot open %s for writting.\n", "/storage/emulated/0/Pictures/TangoCaptures/pic1.ppm" );
        exit(-1);
      }

      fprintf( f0, "P6\n%d %d\n255\n", small_screen_width, small_screen_height);
      fwrite( topdown_pixel, sizeof(unsigned char), small_screen_width*small_screen_height*3, f0);
      fclose( f0 );
  }else {
    printf("Error: can't open directory");
  }

  saveNextFrame = false;
  }

  //change back to normal screen size
    small_screen_height = temp_height;
    small_screen_width = temp_width;

  return true;
}

void SaveFrame(){
  saveNextFrame = true;
//LOGI("true");
}



#ifdef __cplusplus
extern "C" {
#endif
// Tango Service interfaces.
JNIEXPORT jint JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_initialize(
    JNIEnv* env, jobject, jobject activity) {
  TangoErrorType err = TangoData::GetInstance().Initialize(env, activity);
  if (err != TANGO_SUCCESS) {
    if (err == TANGO_INVALID) {
      LOGE("Tango Service version mis-match");
    } else {
      LOGE("Tango Service initialize internal error");
    }
  }
  return static_cast<int>(err);
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_setupConfig(
    JNIEnv*, jobject) {
  if (!TangoData::GetInstance().SetConfig()) {
    LOGE("Tango set config failed");
  }
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_connectCallbacks(
    JNIEnv*, jobject) {
  if (!TangoData::GetInstance().ConnectCallbacks()) {
    LOGE("Tango ConnectCallbacks failed");
  }
}

JNIEXPORT jint JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_connect(
    JNIEnv*, jobject) {
  TangoErrorType err = TangoData::GetInstance().Connect();
  if (err != TANGO_SUCCESS) {
    LOGE("Tango Service connect failed");
  }
  return static_cast<int>(err);
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_setupExtrinsics(
    JNIEnv*, jobject) {
  // The extrinsics can only be queried after connected to service.
  if (!TangoData::GetInstance().SetupExtrinsicsMatrices()) {
    LOGE("Tango set extrinsics failed");
  }
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_disconnect(
    JNIEnv*, jobject) {
  TangoData::GetInstance().Disconnect();
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_freeGLContent(
    JNIEnv*, jobject) {
  if (cam != NULL) {
    delete cam;
  }
  cam = NULL;

  if (pointcloud != NULL) {
    delete pointcloud;
  }
  pointcloud = NULL;

  if (axis != NULL) {
    delete axis;
  }
  axis = NULL;

  if (grid != NULL) {
    delete grid;
  }
  grid = NULL;

  if (frustum != NULL) {
    delete frustum;
  }
  frustum = NULL;

  if (cam_parent_transform != NULL) {
    delete cam_parent_transform;
  }
  cam_parent_transform = NULL;
}
//6.24
// Graphic interfaces.
JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_camerainitGlContent(
    JNIEnv*, jobject) {
  CameraInitGlContent();
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_camerasetupGraphic(
    JNIEnv*, jobject, jint width, jint height) {
  CameraSetupGraphics(width, height);
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_camerarender(
    JNIEnv*, jobject) {
  CameraRenderFrame();
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_connectTexture(
    JNIEnv*, jobject) {
  TangoData::GetInstance().ConnectTexture(video_overlay->GetTextureId());
}

// Graphic interfaces.
JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_initGlContent(
    JNIEnv*, jobject) {
  InitGlContent();
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_setupGraphic(
    JNIEnv*, jobject, jint width, jint height) {
  SetupGraphics(width, height);
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_render(
    JNIEnv*, jobject) {
  RenderFrame();
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_setCamera(
    JNIEnv*, jobject, int camera_index) {
  SetCamera(static_cast<CameraType>(camera_index));
}

JNIEXPORT jint JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_getDirection(
    JNIEnv*, jobject) {
  pthread_mutex_lock(&TangoData::GetInstance().xyzij_mutex);
  int ret_val = TangoData::GetInstance().direction;
  pthread_mutex_unlock(&TangoData::GetInstance().xyzij_mutex);
  return ret_val;
}

// Tango data interfaces.
JNIEXPORT jstring JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_getPoseString(
    JNIEnv* env, jobject) {
  pthread_mutex_lock(&TangoData::GetInstance().pose_mutex);
  std::string ret_string = TangoData::GetInstance().pose_string;
  pthread_mutex_unlock(&TangoData::GetInstance().pose_mutex);
  return (env)->NewStringUTF(ret_string.c_str());
}

JNIEXPORT jstring JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_getEventString(
    JNIEnv* env, jobject) {
  pthread_mutex_lock(&TangoData::GetInstance().event_mutex);
  std::string ret_string = TangoData::GetInstance().event_string;
  pthread_mutex_unlock(&TangoData::GetInstance().event_mutex);
  return (env)->NewStringUTF(ret_string.c_str());
}

JNIEXPORT jstring JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_getVersionNumber(
    JNIEnv* env, jobject) {
  return (env)
      ->NewStringUTF(TangoData::GetInstance().lib_version_string.c_str());
}

JNIEXPORT jint JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_getVerticesCount(
    JNIEnv*, jobject) {
  pthread_mutex_lock(&TangoData::GetInstance().xyzij_mutex);
  int ret_val = TangoData::GetInstance().depth_buffer_size;
  pthread_mutex_unlock(&TangoData::GetInstance().xyzij_mutex);
  return ret_val;
}

JNIEXPORT float JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_getAverageZ(
    JNIEnv*, jobject) {
  pthread_mutex_lock(&TangoData::GetInstance().xyzij_mutex);
  float ret_val = TangoData::GetInstance().depth_average_length;
  pthread_mutex_unlock(&TangoData::GetInstance().xyzij_mutex);
  return ret_val;
}

JNIEXPORT float JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_getFrameDeltaTime(
    JNIEnv*, jobject) {
  pthread_mutex_lock(&TangoData::GetInstance().xyzij_mutex);
  float ret_val = TangoData::GetInstance().depth_frame_delta_time;
  pthread_mutex_unlock(&TangoData::GetInstance().xyzij_mutex);
  return ret_val;
}

// Touching GL interface.
JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_startSetCameraOffset(
    JNIEnv*, jobject) {
  if (cam != NULL) {
    cam_start_angle[0] = cam_cur_angle[0];
    cam_start_angle[1] = cam_cur_angle[1];
    cam_start_dist = cam->GetPosition().z;
  }
}

JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_setCameraOffset(
    JNIEnv*, jobject, float rotation_x, float rotation_y, float dist) {
  if (cam != NULL) {
    cam_cur_angle[0] = cam_start_angle[0] + rotation_x;
    cam_cur_angle[1] = cam_start_angle[1] + rotation_y;
    dist = tango_gl::util::Clamp(cam_start_dist + dist * kZoomSpeed,
                                 kCamViewMinDist, kCamViewMaxDist);
    cam_cur_dist = dist;
  }
}

//6.26
JNIEXPORT void JNICALL
Java_com_projecttango_experiments_nativepointcloud_TangoJNINative_saveFrame(
    JNIEnv*, jobject) {
  SaveFrame();
}

#ifdef __cplusplus
}
#endif
