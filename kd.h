#include <array>
#include <vector>
#include <limits>
#include <memory>
#include <iterator>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>

#define SHRPTR std::shared_ptr
#define WKPTR  std::weak_ptr

template <typename T>
class KDTree
{

public:
    typedef std::vector<T>                     KDPoint;
    typedef std::pair<KDPoint, unsigned>        KDData;
    typedef std::pair<KDData, KDData>          KDPlane;
    typedef typename std::vector<KDData>::iterator KDPointIterator;


    static T getDistance(const KDData &P, const KDData &Q) {
        return sqrtf(getDistanceSq(P, Q));
    }

private:
    int  m_dimension;

    static float getDistanceSq(const KDData &P, const KDData &Q) {
        T Sum = 0;
        int dimension = P.first.size();

        for (unsigned i = 0; i < dimension; i++)
            Sum += (P.first[i] - Q.first[i]) * (P.first[i] - Q.first[i]);

        return Sum;
    }

    static bool intersectPlane(const KDPlane &Region1, const KDPlane &Region2)
    {
        int dimension = Region1.first.first.size();

        for (unsigned i = 0; i < dimension; i++)
            if (Region1.first.first[i] > Region2.second.first[i] || 
                Region1.second.first[i] < Region2.first.first[i])
                return false;

        return true;
    }

    static KDPlane createPlane(const KDData &Point1, const KDData &Point2) {
        
        int dimension = Point1.first.size();
        KDPlane retVal;

        for (unsigned i = 0; i < dimension; i++) {
            retVal.first.first.push_back(std::min(Point1.first[i], Point2.first[i]));
            retVal.second.first.push_back(std::max(Point1.first[i], Point2.first[i]));
        }

        return retVal;
    }

    static KDPlane createPlane(const KDData &Point, T distance) {
        int dimension = Point.first.size();

        KDPlane retVal;

        for (unsigned i = 0; i < dimension; i++) {
            retVal.first.first.push_back(Point.first[i] - distance);
            retVal.second.first.push_back(Point.first[i] + distance);
        }

        return retVal;
    }

    KDPlane createPlane(KDPointIterator &iterBegin,
                          KDPointIterator &iterEnd) {
        KDPlane retVal;

        if (iterBegin != iterEnd) {
            retVal.first = retVal.second = *iterBegin;

            for (KDPointIterator currPoint = 
                    std::next(iterBegin); currPoint != iterEnd; currPoint++) {
                for (unsigned i = 0; i < m_dimension; i++) {
                    retVal.first.first[i]  = std::min(retVal.first.first[i],  (*currPoint).first[i]);
                    retVal.second.first[i] = std::max(retVal.second.first[i], (*currPoint).first[i]);
                }
            }
        }
        else {
            for (unsigned i = 0; i < m_dimension; i++) {
                retVal.first.first.push_back(std::numeric_limits<T>::quiet_NaN());
                retVal.second.first.push_back(std::numeric_limits<T>::quiet_NaN());
            }
        }

        return retVal;
    }

    unsigned diffIndex(const KDData &P, const KDData &Q, unsigned s = 0) {
        for (unsigned i = 0; i < m_dimension; ++i, ++s %= m_dimension)
            if (P.first[s] != Q.first[s])
                return s;

        return m_dimension;
    }

    unsigned diffIndex(const KDPlane &box, unsigned s = 0) {
        return diffIndex(box.first, box.second, s);
    }

protected:

    class KDNode {
    public:
        virtual bool        isInternal() const = 0;
        virtual KDPlane     boundedPlane() const = 0;
        virtual void        nearestNeighbor(const KDData &srcPoint, KDData &nearPoint,
                                     T &minDistance, KDPlane &minRegion) const = 0;
    };


    class KDInnerNode : public KDNode
    {
    private:
        unsigned    m_axis;
        T       m_splitVal;

    public:
        KDPlane      m_boundingBox;
        SHRPTR<KDNode> m_left, m_right;

        KDInnerNode(const T splitVal, const unsigned axis,
                const KDPlane &boundedPlane, SHRPTR<KDNode> Left, 
                SHRPTR<KDNode> Right) :
                                    m_splitVal(splitVal), 
                                    m_axis(axis), 
                                    m_boundingBox(boundedPlane), 
                                    m_left(Left), 
                                    m_right(Right) {}

        unsigned splitAxis() const { 
            return m_axis;
        }

        T splitVal() const {
            return m_splitVal;
        }

        virtual bool isInternal() const override  { 
            return true; 
        }

        virtual KDPlane boundedPlane() const override  { 
            return m_boundingBox; 
        }

        virtual void nearestNeighbor(const KDData &srcPoint, KDData &nearPoint,
                                     T &minDistance, KDPlane &minRegion) 
                                     const override {

            if (intersectPlane(m_left->boundedPlane(), minRegion))
                m_left->nearestNeighbor(srcPoint, nearPoint, minDistance, minRegion);

            if (intersectPlane(m_right->boundedPlane(), minRegion))
                m_right->nearestNeighbor(srcPoint, nearPoint, minDistance, minRegion);
        }
    };

    class KDLeafNode : public KDNode {

    private:
        KDData m_pointCoords;

    public:
        WKPTR<KDLeafNode> m_Prev, m_Next;

        KDLeafNode(const KDData &Point) : m_pointCoords(Point) { }

        KDLeafNode(const KDData &Point, WKPTR<KDLeafNode> Prev, 
                WKPTR<KDLeafNode> Next) :
                    m_pointCoords(Point),
                    m_Prev(Prev),
                    m_Next(Next) {}

        const KDData pointCoords() const { return m_pointCoords; }

        virtual bool isInternal() const override  { return false; }

        virtual KDPlane boundedPlane() const override  {
            return { m_pointCoords, m_pointCoords };
        }

        virtual void nearestNeighbor(const KDData &srcPoint,
                                     KDData &nearPoint,
                                     T &minDistance, 
                                     KDPlane &minRegion) const override {
                T currDistance = getDistance(srcPoint, m_pointCoords);

            if (currDistance < minDistance) {
                nearPoint   = m_pointCoords;
                minDistance = currDistance;
                minRegion   = createPlane(srcPoint, minDistance);
            }
        }
    };

    T getAxisMedian(KDPointIterator iterBegin,
                         KDPointIterator iterEnd,
                         const unsigned num) {
        // Sorting on the basis of split axis
        std::sort(iterBegin, iterEnd, [num](const KDData &A, const KDData &B) {
                return A.first[num] < B.first[num];
        });

        size_t numPoints = iterEnd - iterBegin;

        T median = (*(iterBegin + numPoints / 2)).first[num];

        if (numPoints % 2 == 0)
            median = (median + (*(iterBegin + numPoints / 2 - 1)).first[num]) / 2.0f;

        if (median == (*(iterEnd - 1)).first[num] && median != (*iterBegin).first[num])
        {
            KDPointIterator medianIter = iterBegin + numPoints / 2;

            while (median == (*(--medianIter)).first[num]);

            median = (median + (*medianIter).first[num]) / 2.0f;
        }

        return median;
    }

    SHRPTR<KDNode> createKDTree(KDPointIterator iterBegin,
                                        KDPointIterator iterEnd,
                                        SHRPTR<KDLeafNode> &lastLeaf,
                                        unsigned Depth = 0)
    {
        // When size of vector is 1, it is the leaf node, else internal nodes
        if (iterEnd - iterBegin == 1)
        {
            SHRPTR<KDLeafNode> retNode(new KDLeafNode(*iterBegin));

            if (lastLeaf)
            {
                lastLeaf->m_Next = retNode;
                retNode->m_Prev = lastLeaf;
            }
            else
                m_firstLeaf = retNode;

            lastLeaf = retNode;

            return retNode;
        }
        else if (iterEnd - iterBegin == 2)
        {
            KDData point0 = *iterBegin, point1 = *(std::next(iterBegin));
            unsigned splitAxis = diffIndex(point0, point1, Depth % m_dimension);

            if (point0.first[splitAxis] > point1.first[splitAxis])
                std::swap(point0, point1);

            SHRPTR<KDLeafNode> Left(new KDLeafNode(point0)), 
                Right(new KDLeafNode(point1));

            if (lastLeaf)
            {
                lastLeaf->m_Next = Left;
                Left->m_Prev = lastLeaf;
            }
            else
                m_firstLeaf = Left;

            Left->m_Next = Right;
            Right->m_Prev = Left;

            lastLeaf = Right;

            SHRPTR<KDInnerNode> retNode(new KDInnerNode((
                            point0.first[splitAxis] + point1.first[splitAxis]) / 2.0f, splitAxis,
                            createPlane(point0, point1), Left, Right));

            return retNode;
        }
        else {
            KDPlane   boundedPlane = createPlane(iterBegin, iterEnd);
            unsigned splitAxis   = diffIndex(boundedPlane, Depth%m_dimension);
            T    median      = getAxisMedian(iterBegin, iterEnd, splitAxis);
            size_t   numPoints   = iterEnd - iterBegin;

            KDPointIterator lastMedianLoc = iterBegin + numPoints / 2;

            if ((*lastMedianLoc).first[splitAxis] != (*std::prev(iterEnd)).first[splitAxis])
                while ((*(++lastMedianLoc)).first[splitAxis] == median);
            else
                while ((*std::prev(lastMedianLoc)).first[splitAxis] == median)
                    lastMedianLoc--;

            SHRPTR<KDNode> Left, Right;
            
            Left  = createKDTree(iterBegin, lastMedianLoc, lastLeaf, Depth + 1);
            Right = createKDTree(lastMedianLoc, iterEnd,   lastLeaf, Depth + 1);

            SHRPTR<KDInnerNode> retNode(new KDInnerNode(median,
                        splitAxis, boundedPlane, Left, Right));

            return retNode;
        }
    }

    KDData semiNearestNeighborPoint(const KDData &srcPoint) const {
        SHRPTR<KDNode> node(m_root);
        SHRPTR<KDInnerNode> iNode;

        while (node->isInternal())
        {
            iNode = std::static_pointer_cast<KDInnerNode>(node);

            node = (srcPoint.first[iNode->splitAxis()] <= iNode->splitVal()) ? 
                iNode->m_left : iNode->m_right;
        }

        SHRPTR<KDLeafNode> retval = std::static_pointer_cast<KDLeafNode>(node);

        return retval->pointCoords();
    }

    SHRPTR<KDNode>          m_root;
    WKPTR<KDLeafNode>       m_firstLeaf;
    std::ofstream           myfile;

public:

    KDTree(int dim) {
        m_dimension = dim;
    }       
    
    KDTree(std::vector<KDData> &Points, int dim) {
        m_dimension = dim;
        insert(Points);
    }

    void clear() {
        m_root.reset();
    }

    ~KDTree() {
        m_root.reset();
    }

    KDPlane boundedPlane() const
    {
        if (m_root)
            return m_root->boundedPlane();
        else {
            KDData P1, P2;

            for (int i = 0; i < m_dimension; i++){
                P1.first.push_back(std::numeric_limits<T>::quiet_NaN());
                P2.first.push_back(std::numeric_limits<T>::quiet_NaN());
            }

            return KDPlane(P1, P2);
        }
    }

    void insert(std::vector<KDData> &Points) {
        this->clear();

        if (Points.size() > 0) {

            sort(Points.begin(), Points.end());
            
            KDPointIterator it = 
                                unique(Points.begin(), Points.end());
            
            Points.resize(distance(Points.begin(), it));
            Points.shrink_to_fit();

            SHRPTR<KDLeafNode> dummy;

            m_root = createKDTree(Points.begin(), Points.end(), dummy);
        }
    }

    void rebuild(std::vector<KDData> &points) {
        insert(points);
    }

    bool nearestNeighbor(const KDData &srcPoint, KDData &nearPoint) const
    {
        bool retVal = (m_root != nullptr);

        if (!m_root) {
            for(unsigned int i = 0; i < nearPoint.first.size(); i++)
            nearPoint.first.push_back(std::numeric_limits<T>::quiet_NaN());
        }
        else {
            nearPoint = semiNearestNeighborPoint(srcPoint);

            T minDistance = getDistance(srcPoint, nearPoint);

            KDPlane thisBox = createPlane(srcPoint, minDistance);

            m_root->nearestNeighbor(srcPoint, nearPoint, minDistance, thisBox);
        }

        return retVal;
    }

    void printKDTree(SHRPTR<KDNode> node, unsigned depth = 0)
    {
        if (node == NULL)
            myfile << "NULL" << std::endl;
        else
        {
            if (node->isInternal())
            {
                SHRPTR<KDInnerNode> iNode = std::static_pointer_cast
                    <KDInnerNode>(node);

                printKDTree(iNode->m_left, depth + 1);
                printKDTree(iNode->m_right, depth + 1);
            }
            else
            {
                SHRPTR<KDLeafNode> lNode = std::static_pointer_cast
                    <KDLeafNode>(node);

                KDData point = lNode->pointCoords();

                std::stringstream ss;

                ss << point.second << " ";

                for (unsigned i = 0; i < m_dimension; i++)
                    ss << std::setprecision(17) << point.first[i] << " ";

                myfile << ss.str() << std::endl;;
            }
        }
    }

    void printKDTree(const std::string& outputFile) {
        myfile.open (outputFile.c_str(),  std::ofstream::out);
        printKDTree(m_root, 0);
    }
};




