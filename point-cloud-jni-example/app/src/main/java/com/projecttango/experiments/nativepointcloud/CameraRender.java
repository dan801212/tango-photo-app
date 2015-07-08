package com.projecttango.experiments.nativepointcloud;

/**
 * Created by dan on 6/24/15.
 */

import android.opengl.GLSurfaceView;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;
import android.os.Environment;
import android.widget.Toast;
import android.util.Log;

import java.io.File;

public class CameraRender implements GLSurfaceView.Renderer{
    public void onDrawFrame(GL10 gl) {
        TangoJNINative.camerarender();
    }

    public void onSurfaceChanged(GL10 gl, int width, int height) {
        TangoJNINative.camerasetupGraphic(width, height);
    }

    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        TangoJNINative.camerainitGlContent();
        TangoJNINative.connectTexture();

        try {
            // Create/access a pictures subdirectory.
            File directory = new File(
                    Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES),
                    "TangoCaptures");
            if (!directory.mkdirs() && !directory.isDirectory()) {

                return;
            }
            Log.d("Activity", " "+directory);
        }catch (Exception e) {
            e.printStackTrace();
        }
    }
}
