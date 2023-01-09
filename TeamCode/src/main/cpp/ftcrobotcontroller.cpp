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




#include "helperthings.h"

bidirectionalVectorPiece<weightedPoint>* lastPlacedPoint = nullptr;
BidirectionalVectorWithStartAndCheckpoint<weightedPoint>* pts = new BidirectionalVectorWithStartAndCheckpoint<weightedPoint>();

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


extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_SharedCode_GraphManager_PlacePoint(JNIEnv *env, jobject thiz,
                                                                       jdouble x, jdouble y) {
    //PlacePoint()
    if (x < 0.0)
    {
        x = 0.0;
    }
    pts->currentPiece = lastPlacedPoint;


    tryToPut:
    if (x > pts->currentPiece->value.time)
    {
        if (pts->willBeLastPiece() || x < pts->currentPiece->nextPiece->value.time)
        {
            pts->pushToNext(weightedPoint{x, y, 1});
            lastPlacedPoint = pts->currentPiece;
            //FillMemory((continguousArrayOfPtsPointers + (int)retPlaced->value.time), (int)retPlaced->nextPiece->value.time - (int)retPlaced->value.time, (int)retPlaced);
            //continguousArrayOfPtsPointers[pt.x] = retPlaced;
        }
        else
        if (x == pts->currentPiece->nextPiece->value.time)
        {
            // equal time to next piece
            pts->selectNext();
            pts->currentPiece->value.value = y;
            pts->currentPiece->value.weight = 1;
            lastPlacedPoint = pts->currentPiece;
            //pts->selectNext();
        }
        else
        if (x > pts->currentPiece->nextPiece->value.time)
        {
            // goes past the next piece
            pts->selectNext();
            goto tryToPut;
        }
    }
    else
    if (x < pts->currentPiece->value.time)
    {
        //pts->setToStart();
        //goto tryToPut;
        // change this to do the thing above, but in reverse.
        if (pts->willBeFirstPiece() || x > pts->currentPiece->prevPiece->value.time)
        {
            pts->pushToPrev(weightedPoint{x, y, 1});
            lastPlacedPoint = pts->currentPiece;
            //FillMemory((continguousArrayOfPtsPointers + (int)retPlaced->value.time), (int)retPlaced->nextPiece->value.time - (int)retPlaced->value.time, (int)retPlaced);
            //continguousArrayOfPtsPointers[pt.x] = retPlaced;
        }
        else
        if (x == pts->currentPiece->prevPiece->value.time)
        {
            // equal time to prev piece
            pts->selectPrev();
            pts->currentPiece->value.value = y;
            pts->currentPiece->value.weight = 1;
            lastPlacedPoint = pts->currentPiece;
            //pts->selectPrev();
        }
        else
        if (x < pts->currentPiece->prevPiece->value.time)
        {
            // goes past the prev piece
            pts->selectPrev();
            goto tryToPut;
        }
    }
    else
    {
        // equal time to current piece
        pts->currentPiece->value.value = y;
        pts->currentPiece->value.weight = 1;
        lastPlacedPoint = pts->currentPiece;
        //pts->selectNext();
    }
    //return lastPlacedPoint;
    return;
}