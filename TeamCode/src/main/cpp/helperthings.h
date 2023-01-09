#pragma once


struct dPoint
{
    double time = 0;
    double value = 0;
};

struct weightedPoint
{
    double time = 0;
    double value = 0;
    double weight = 1;

    operator dPoint()
    {
        return dPoint{time, value};
    }
};


double linearInterpolator(dPoint p1, dPoint p2, double xPosBetween)
{
    double percentP2 = (xPosBetween - p1.time) / (p2.time - p1.time);
    return (percentP2 * p2.value + (1.0 - percentP2) * p1.value);
}

double weightedInterpolator(weightedPoint p1, weightedPoint p2, double xPosBetween)
{
    double percentP2 = (xPosBetween - p1.time) / (p2.time - p1.time);
    double percentP1 = 1.0 - percentP2;
    percentP1 *= p1.weight;
    percentP2 *= p2.weight;
    percentP1 = percentP1 / (percentP1 + percentP2);
    percentP2 = 1.0 - percentP1; // could also be defined as percentP2 = percentP2 / (percentP1 + percentP2), but only if this line of code is before the previous line because it needs the previous value of percentP1

    return (percentP2 * p2.value + percentP1 * p1.value);
}

template<typename T>
class vectorPiece
{
public:
    T value = {};
    vectorPiece<T>* nextPiece = 0;
    vectorPiece() {}
    vectorPiece(T initValue, vectorPiece<T>* nextPiece) : value(initValue), nextPiece(nextPiece) {}
};

template<typename T>
class bidirectionalVectorPiece
{
public:
    T value = {};
    bidirectionalVectorPiece<T>* prevPiece = 0;
    bidirectionalVectorPiece<T>* nextPiece = 0;
    bidirectionalVectorPiece() {}
    bidirectionalVectorPiece(T initValue, bidirectionalVectorPiece<T>* initPrevPiece, bidirectionalVectorPiece<T>* initNextPiece) : value(initValue), prevPiece(initPrevPiece), nextPiece(initNextPiece) {}
};

template<typename T>
class BidirectionalVectorWithStartAndCheckpoint
{
public:
    bidirectionalVectorPiece<T>* currentPiece = 0;
    bidirectionalVectorPiece<T>* startPointer = 0;
    bidirectionalVectorPiece<T>* checkPoint = 0;

    BidirectionalVectorWithStartAndCheckpoint(T firstValue)
    {
        currentPiece = new bidirectionalVectorPiece<T>{ firstValue, NULL, NULL };
        startPointer = currentPiece;
        checkPoint = currentPiece;
    }
    BidirectionalVectorWithStartAndCheckpoint()
    {
        currentPiece = new bidirectionalVectorPiece<T>{};
        startPointer = currentPiece;
        checkPoint = currentPiece;
    }
    ~BidirectionalVectorWithStartAndCheckpoint()
    {
        bidirectionalVectorPiece<T>* currentToDelete;
        currentPiece = startPointer;
        while (currentPiece != NULL)
        {
            currentToDelete = currentPiece;
            currentPiece = currentPiece->nextPiece;
            delete currentToDelete;
        }
    }

    void pushNextFromCurrent(T value)
    {
        bidirectionalVectorPiece<T>* oldNext = currentPiece->nextPiece;
        currentPiece->nextPiece = new bidirectionalVectorPiece<T>( value, currentPiece, oldNext );
        oldNext->prevPiece = currentPiece->nextPiece;
    }

    bool selectNext()
    {
        if (currentPiece->nextPiece)
        {
            currentPiece = currentPiece->nextPiece;
            return true;
        }
        return false;
    }

    void pushToNext(T value)
    {
        bidirectionalVectorPiece<T>* oldNext = currentPiece->nextPiece;
        currentPiece->nextPiece = new bidirectionalVectorPiece<T>( value, currentPiece, oldNext );
        if(oldNext)
            oldNext->prevPiece = currentPiece->nextPiece;
        currentPiece = currentPiece->nextPiece; // selectNext()
    }

    T next()
    {
        // For safety, it is recomended to call isLastPiece, before calling this
        T Treturn = currentPiece->value;
        if (currentPiece->nextPiece)
        {
            currentPiece = currentPiece->nextPiece;
        }
        return Treturn;
    }

    void pushPrevFromCurrent(T value)
    {
        bidirectionalVectorPiece<T>* oldPrev = currentPiece->prevPiece;
        currentPiece->prevPiece = new bidirectionalVectorPiece<T>( value, oldPrev, currentPiece );
        oldPrev->nextPiece = currentPiece->prevPiece;
    }

    bool selectPrev()
    {
        if (currentPiece->prevPiece)
        {
            currentPiece = currentPiece->prevPiece;
            return true;
        }
        return false;
    }

    void pushToPrev(T value)
    {
        bidirectionalVectorPiece<T>* oldPrev = currentPiece->prevPiece;
        currentPiece->prevPiece = new bidirectionalVectorPiece<T>( value, oldPrev, currentPiece );
        if(oldPrev)
            oldPrev->nextPiece = currentPiece->prevPiece;
        currentPiece = currentPiece->prevPiece; // selectPrev()
    }

    T Prev()
    {
        // For safety, it is recomended to check wasLastPiece, before calling this
        T Treturn = currentPiece->value;
        if (currentPiece->prevPiece)
        {
            currentPiece = currentPiece->prevPiece;
        }
        return Treturn;
    }

    void setToStart()
    {
        currentPiece = startPointer;
    }

    void setToCheckpoint()
    {
        currentPiece = checkPoint;
    }

    bool willBeLastPiece()
    {
        return !(bool)currentPiece->nextPiece;
    }

    bool willBeFirstPiece()
    {
        return !(bool)currentPiece->prevPiece;
    }

};

template<typename T>
class loopingVector
{
    // Keeps track of current position, Loops back to start from end
public:
    vectorPiece<T>* currentPiece = 0;
    //vectorPiece<T>* startPiece = 0; Planned to have an inheriting class that implements a start.		May also have one that has a stored size and one that alows access to an Nth term

    loopingVector(T firstValue)
    {
        currentPiece = new vectorPiece<T>{ firstValue, 0 };
        currentPiece->nextPiece = currentPiece;
    }

    loopingVector()
    {
        currentPiece = new vectorPiece<T>{};
        currentPiece->nextPiece = currentPiece;
    }

    ~loopingVector()
    {
        vectorPiece<T>* start = currentPiece;
        vectorPiece<T>* currentToDelete;
        do
        {
            currentToDelete = currentPiece;
            currentPiece = currentPiece->nextPiece;
            delete currentToDelete;

        } while (currentPiece != start);
    }

    void pushFromCurrent(T value)
    {
        vectorPiece<T>* oldNext = currentPiece->nextPiece;
        currentPiece->nextPiece = new vectorPiece<T>{ value, oldNext };
    }

    void selectNext()
    {
        currentPiece = currentPiece->nextPiece;
    }

    void pushToNext(T value)
    {
        vectorPiece<T>* oldNext = currentPiece->nextPiece;
        currentPiece->nextPiece = new vectorPiece<T>{ value, oldNext };
        currentPiece = currentPiece->nextPiece; // selectNext()
    }

    T next()
    {
        T Treturn = currentPiece->value;
        currentPiece = currentPiece->nextPiece;
        return Treturn;
    }

    bool willBeLastPiece()
    {
        // if the next piece that will be returned is the last piece before it loops back to the start
        return this->currentPiece->nextPiece == this->startPointer;
    }
    bool willBeLastPiece(size_t piecesForward)
    {
        vectorPiece<T>* checkingPiece = this->currentPiece;
        for (size_t i = 0; i < piecesForward; ++i)
        {
            checkingPiece = checkingPiece->nextPiece;
        }
        return checkingPiece->nextPiece == this->startPointer;
    }
};

template<typename T>
class loopingWithStart : public loopingVector<T>
{
public:
    vectorPiece<T>* startPointer;
    vectorPiece<T>* checkpointPointer;
    void setToCheckpoint()
    {
        this->currentPiece = checkpointPointer;
    }
    loopingWithStart(T firstValue) : loopingVector<T>(firstValue)
    {
        startPointer = this->currentPiece;
    }

    loopingWithStart() : loopingVector<T>()
    {
        startPointer = this->currentPiece;
    }

    T start()
    {
        this->currentPiece = startPointer->nextPiece;
        return startPointer->value;
    }

    void setToStart()
    {
        this->currentPiece = startPointer;
    }

    bool isFirstPiece()
    {
        return this->currentPiece == startPointer;
    }

    bool willBeLastPiece()
    {
        // if the next piece that will be returned is the last piece before it loops back to the start
        return this->currentPiece->nextPiece == startPointer;
    }

    bool willBeLastPiece(size_t piecesForward)
    {
        vectorPiece<T>* checkingPiece = this->currentPiece;
        for (size_t i = 0; i < piecesForward; ++i)
        {
            checkingPiece = checkingPiece->nextPiece;
        }
        return checkingPiece->nextPiece == startPointer;
    }
};