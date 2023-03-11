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




#include "point.h"

listPiece<Vector2D>* lastPlacedPoint = nullptr;
listManager<Vector2D>* pts = new listManager<Vector2D>(Vector2D(12, 12), 0); // starts in the middle of tile 0,0
#if 0
char bytes[2000];

std::fstream graphFile;

extern "C"
JNIEXPORT jboolean JNICALL
Java_org_firstinspires_ftc_teamcode_SharedCode_GraphManager_SaveGraph(JNIEnv *env, jobject thiz) {
    // SaveGraph()
    graphFile.open("/storage/emulated/0/FIRST/graph", std::ios_base::binary | std::ios_base::out);

    if (!graphFile.is_open())
    {
        return false;
    }

    size_t szWave = 0;
    pts->setToStart();
    bidirectionalVectorPiece<weightedPoint>* current = pts->startPointer;
    do
    {
        // Get number of points. Maybe change this to have the vector keep track.
        current = current->nextPiece;
        ++szWave;
    } while (nullptr != current);
    pts->setToStart();
    weightedPoint* contigArray = new weightedPoint[szWave];
    weightedPoint newPoint;
    double lastX = pts->currentPiece->value.time, newX = 0;
    for (size_t i=0;i<szWave;++i)
    {
        newPoint = pts->next();
        double temp = newPoint.time;
        newPoint.time -= lastX;
        lastX = temp;

        newPoint.value = -newPoint.value; // To make the upward direction positive instead of negative

        contigArray[i] = newPoint;

    }
    /*
    DWORD bytesWritten = 0;
    WriteFile(hFile, contigArray, sizeof(weightedPoint) * szWave, &bytesWritten, NULL);
    CloseHandle(hFile);
    */
    graphFile.write((char*)contigArray, sizeof(weightedPoint) * szWave);
    graphFile.close();
    delete[] contigArray;

    return true; // maybe should change to return if the file was successfully saved
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_org_firstinspires_ftc_teamcode_SharedCode_GraphManager_LoadGraph(JNIEnv *env, jobject thiz) {
    // TODO: implement LoadGraph()
    graphFile.open("/storage/emulated/0/FIRST/graph", std::ios_base::binary | std::ios_base::in);

    if (!graphFile.is_open())
    {
        return false;
    }

    std::streampos szFile = graphFile.tellg();
    graphFile.seekg( 0, std::ios::end );
    szFile = graphFile.tellg() - szFile;
    size_t szWave = szFile / sizeof(weightedPoint);
    weightedPoint* contigArray = new weightedPoint[szWave];

    graphFile.read((char*)contigArray, szFile);
    graphFile.close();
    /*
    DWORD bytesRead = 0;
    ReadFile(hFile, contigArray, szFile.LowPart, &bytesRead, NULL); // only reads the file up to the 32 bit unsigned int limit number of bytes (I might add support for it to work with larger files later)
*/
    delete pts;

    weightedPoint newPoint = contigArray[0];
    newPoint.value *= -1;
    double lastX = newPoint.time;
    pts = new BidirectionalVectorWithStartAndCheckpoint<weightedPoint>(newPoint);


    for (size_t i = 1;i<szWave;++i)
    {
        newPoint = contigArray[i];
        newPoint.value *= -1;
        newPoint.time += lastX;
        lastX = newPoint.time;
        pts->pushToNext(newPoint);
    }
    pts->setToStart();
    // TODO: set the checkpoint of the loaded pts

    lastPlacedPoint = pts->startPointer; // make this go to the checkpoint (instead of the start)
    delete[] contigArray;

}


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

#endif
extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_SharedCode_GraphManager_PlacePoint(JNIEnv *env, jobject thiz,
                                                                       jdouble x, jdouble y, jdouble time) {
    //PlacePoint()
    pts->insertPoint(Vector2D(x, y), time);
    return;
}

Vector2D outPoint = {};

extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_SharedCode_GraphManager_getPoint(JNIEnv *env, jobject thiz,
                                                                     jdouble time) {
    // TODO: implement getPoint()
    outPoint = pts->getPoint(time);
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_SharedCode_GraphManager_getXOfPoint(JNIEnv *env, jobject thiz) {
    // TODO: implement getXOfPoint()
    return outPoint.x;
}
extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_SharedCode_GraphManager_getYOfPoint(JNIEnv *env,
                                                                         jobject thiz) {
    // TODO: implement getYoOfPoint()
    return outPoint.y;
}
extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_SharedCode_GraphManager_getLastPointTime(JNIEnv *env,
                                                                             jobject thiz) {
    // TODO: implement getLastPointTime()
    double ret = 0;
    listPiece<Vector2D>* current = pts->getCurrentListPiece();
    while (current->getNext()) // select the last point
        current = current->getNext();
    return current->time; // return its time
}
extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_SharedCode_GraphManager_deleteAllPoints(JNIEnv *env,
                                                                            jobject thiz,
                                                                            jdouble new_start_x,
                                                                            jdouble new_start_y,
                                                                            jdouble current_time) {
    // TODO: implement deleteAllPoints()
    pts->replaceList(Vector2D(new_start_x, new_start_y), current_time);
}