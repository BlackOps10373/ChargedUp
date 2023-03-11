#pragma once
#if _DEBUG
#include <iostream>
#endif

template<typename T>
T interpolate(T value1, T value2, double percentBetween)
{
    return (value1 * (1.0 - percentBetween)) + (value2 * percentBetween);
}


template<typename T>
class listManager;

template<typename T>
class listPiece
{
    friend listManager<T>;
public:
    T value;
    double time;
private:
    listPiece<T>* next = 0;
    listPiece<T>* prev = 0;

public:
    listPiece()
            : value()
    {
    }
    ~listPiece()
    {
        if (next)
        {
            next->prev = nullptr;
            delete next;
        }
        if (prev)
        {
            prev->next = nullptr;
            delete prev;
        }
    }

    void insertNext(T value, double time)
    {
        if (!next)
        {
            next = new listPiece<T>();
            next->value = value;
            next->time = time;
            next->prev = this;
        }
        else
        {
            listPiece<T>* oldNext = next;
            next = new listPiece<T>();
            next->value = value;
            next->time = time;
            next->next = oldNext;
            oldNext->prev = next;
            next->prev = this;
        }
    }
    void insertPrev(T value, double time)
    {
        if (!prev)
        {
            prev = new listPiece<T>();
            prev->value = value;
            prev->time = time;
            prev->next = this;
        }
        else
        {
            listPiece<T>* oldPrev = prev;
            prev = new listPiece<T>();
            prev->value = value;
            prev->time = time;
            prev->prev = oldPrev;
            oldPrev->next = prev;
            prev->next = this;
        }
    }
    inline listPiece<T>* const getNext() const
    {
        return next;
    }
    inline listPiece<T>* const getPrev() const
    {
        return prev;
    }

    typedef listPiece<T>* pointEntry;


private:
    listPiece<T>* insertPoint(T point, double time, pointEntry lastEntry)
    {
#if _DEBUG
        pointEntry startedAt = lastEntry;
#endif

        /*
        if (!pts)
        {
            pts = new listPiece<Vector2D>();
            pts->value = point;
            pts->time = time;
            lastEntry = pts;
        }
        */
        // point is equal to current
        if (time == lastEntry->time)
        {
            lastEntry->value = point;
            return lastEntry;
        }

        if (time > lastEntry->time)
        {
            // select the entry that is at the end of the list or before the one that has a lower or equal time value than the adding point
            while (lastEntry->getNext() && time >= lastEntry->getNext()->time)
            {
                lastEntry = lastEntry->getNext();

#if _DEBUG
                if (lastEntry == startedAt)
				{
					std::cout << "Error: List Looped";
					return lastEntry;
				}
#endif
            }
            if (time == lastEntry->time) // if equal in time, replace it
            {
                lastEntry->value = point;
                return lastEntry;
            }
            if (time > lastEntry->time) // if still greater, insert here (it must be less than the next one, but greater than the current to be inserted between)
            {
                lastEntry->insertNext(point, time);
                lastEntry = lastEntry->getNext();
                return lastEntry;
            }
        }

        if (time < lastEntry->time)
        {
            // select the entry that is at the end of the list or before the one that has a greater or equal time value than the adding point
            while (lastEntry->getPrev() && time <= lastEntry->getPrev()->time)
            {
                lastEntry = lastEntry->getPrev();

#if _DEBUG
                if (lastEntry == startedAt)
				{
					std::cout << "Error: List Looped";
					return lastEntry;
				}
#endif
            }
            if (time == lastEntry->time) // if equal in time, replace it
            {
                lastEntry->value = point;
                return lastEntry;
            }
            if (time < lastEntry->time) // if still less, insert here (it must be greater than the next one, but less than the current to be inserted between)
            {
                lastEntry->insertPrev(point, time);
                lastEntry = lastEntry->getPrev();
                return lastEntry;
            }
        }
    }
public:
    inline void insertPoint(T point, double time)
    {
        insertPoint(point, time, this);
    }
private:
    T getPoint(double time, pointEntry lastEntry, pointEntry* newLastEntry)
    {
        if (time == lastEntry->time)
        {
            return lastEntry->value;
        }
        if (time > lastEntry->time)
        {
            while (lastEntry->getNext() && time > lastEntry->getNext()->time)
            {
                lastEntry = lastEntry->getNext();
            }

            if (newLastEntry)
            {
                *newLastEntry = lastEntry;
            }

            if (lastEntry->getNext())
            {
                return interpolate(lastEntry->value, lastEntry->getNext()->value, (time - lastEntry->time) / (lastEntry->getNext()->time - lastEntry->time));
            }
            else
            {
                return lastEntry->value;
            }
        }

        if (time < lastEntry->time)
        {
            while (lastEntry->getPrev() && time < lastEntry->getPrev()->time)
            {
                lastEntry = lastEntry->getPrev();
            }

            if (newLastEntry)
            {
                *newLastEntry = lastEntry;
            }

            if (lastEntry->getPrev())
            {
                return interpolate(lastEntry->getPrev()->value, lastEntry->value, (time - lastEntry->getPrev()->time) / (lastEntry->time - lastEntry->getPrev()->time));
            }
            else
            {
                return lastEntry->value;
            }
        }

    }
public:
    T getPoint(double time)
    {
        return getPoint(time, this, nullptr);
    }
};

template<typename T>
class listManager
{
private:
    listPiece<T>* lastPiece;
public:
    listManager()
            : lastPiece(new listPiece<T>())
    {
    }
    listManager(T&& _init, double _time)
            : lastPiece(new listPiece<T>(_init, _time))
    {
    }
    inline void insertPoint(T point, double time)
    {
        lastPiece = lastPiece->insertPoint(point, time, lastPiece);
    }
    inline T getPoint(double time)
    {
        return lastPiece->getPoint(time, lastPiece, &lastPiece);
    }
    listPiece<T>* getCurrentListPiece() const
    {
        return lastPiece;
    }

    void replaceList(T&& _newStart, double time)
    {
        delete lastPiece; // delete whole old list
        lastPiece = new listPiece<T>(_newStart, time); // create new list start
    }
};


class Vector2D
{
public:
    double x = 0, y = 0;
    Vector2D()
    {
    }
    Vector2D(double _x, double _y)
            : x(_x), y(_y)
    {

    }

    Vector2D operator+(Vector2D other)
    {
        other.x += x;
        other.y += y;

        return other;
    }
    Vector2D operator-(Vector2D other)
    {
        other.x -= x;
        other.y -= y;
        return other;
    }
    Vector2D operator*(Vector2D other)
    {
        other.x *= x;
        other.y *= y;
        return other;
    }
    Vector2D operator/(Vector2D other)
    {
        other.x /= x;
        other.y /= y;
        return other;
    }
    Vector2D& operator+=(Vector2D& other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }
    Vector2D& operator-=(Vector2D& other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    Vector2D& operator*=(Vector2D& other)
    {
        x *= other.x;
        y *= other.y;
        return *this;
    }
    Vector2D& operator/=(Vector2D& other)
    {
        x /= other.x;
        y /= other.y;
        return *this;
    }
    Vector2D operator+(double other)
    {
        Vector2D ret = *this;
        ret.x += other;
        ret.y += other;

        return ret;
    }
    Vector2D operator-(double other)
    {
        Vector2D ret = *this;
        ret.x -= other;
        ret.y -= other;

        return ret;
    }
    Vector2D operator*(double other)
    {
        Vector2D ret = *this;
        ret.x *= other;
        ret.y *= other;

        return ret;
    }
    Vector2D operator/(double other)
    {
        Vector2D ret = *this;
        ret.x /= other;
        ret.y /= other;

        return ret;
    }
    Vector2D& operator+=(double other)
    {
        x += other;
        y += other;
        return *this;
    }
    Vector2D& operator-=(double other)
    {
        x -= other;
        y -= other;
        return *this;
    }
    Vector2D& operator*=(double other)
    {
        x *= other;
        y *= other;
        return *this;
    }
    Vector2D& operator/=(double other)
    {
        x /= other;
        y /= other;
        return *this;
    }
};

void insertPoint(Vector2D point, double time);
