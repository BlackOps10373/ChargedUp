#include <jni.h>

// Write C++ code here.
//
// Do not forget to dynamically load the C++ library into your application.
//
// For instance,
//
// In MainActivity.java:
//    static {
//       System.loadLibrary("ftcrobotcontroller");
//    }
//
// Or, in MainActivity.kt:
//    companion object {
//      init {
//         System.loadLibrary("ftcrobotcontroller")
//      }
//    }

#include <fstream>

char bytes[2000];

extern "C"
JNIEXPORT jboolean JNICALL
Java_org_firstinspires_ftc_teamcode_FileTest_readFile(JNIEnv *env, jobject thiz) {
    // TODO: implement readFile()

    std::ifstream file;
    file.open("/storage/emulated/0/FIRST/test.txt", std::ios_base::binary);
    file.read(bytes, 2000);

    //FILE *test = fopen("/storage/emulated/0/FIRST/test.txt", "w+");


    return bytes[0] == 'T';
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_org_firstinspires_ftc_teamcode_FileTest_testWrite(JNIEnv *env, jobject thiz) {
    // TODO: implement testWrite()
    std::ofstream file;
    file.open("/storage/emulated/0/FIRST/test.txt", std::ios_base::binary);
    file.write("Test!", 6);
    return file.is_open();
}
