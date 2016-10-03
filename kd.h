#include <array>
#include <vector>
#include <limits>
#include <memory>
#include <iterator>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>

#define PRINTPRECISE    std::setprecision(17)
 
#define SHRPTR          std::shared_ptr
#define WKPTR           std::weak_ptr

template <typename T>
class KDTree
{

public:
    typedef std::vector<T>                      KDData;
    typedef std::pair<KDData, unsigned>         KDPoint;
    typedef std::pair<KDPoint, KDPoint>         KDPlane;
    typedef typename std::vector<KDPoint>::iterator KDPointIterator;

    static T getDistance(const KDPoint &P, const KDPoint &Q) {
        return sqrtf(getDistanceSq(P, Q));
    }

private:
    int  m_dimension;

    static float getDistanceSq(const KDPoint &P, const KDPoint &Q) {
        T sum = 0;
        int dimension = P.first.size();

        for (unsigned i = 0; i < dimension; i++)
            sum += (P.first[i] - Q.first[i]) * (P.first[i] - Q.first[i]);

        return sum;
    }

    static bool intersectPlane(const KDPlane &plane1, const KDPlane &plane2)
    {
        int dimension = plane1.first.first.size();

        for (unsigned i = 0; i < dimension; i++)
            if (plane1.first.first[i] > plane2.second.first[i] || 
                plane1.second.first[i] < plane2.first.first[i])
                return false;

        return true;
    }

    static KDPlane createPlane(const KDPoint &point1, const KDPoint &point2) {
        
        int dimension = point1.first.size();
        KDPlane retVal;

        for (unsigned i = 0; i < dimension; i++) {
            retVal.first.first.push_back(std::min(point1.first[i], point2.first[i]));
            retVal.second.first.push_back(std::max(point1.first[i], point2.first[i]));
        }

        return retVal;
    }

    static KDPlane createPlane(const KDPoint &point, T distance) {
        int dimension = point.first.size();

        KDPlane retVal;

        for (unsigned i = 0; i < dimension; i++) {
            retVal.first.first.push_back(point.first[i] - distance);
            retVal.second.first.push_back(point.first[i] + distance);
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

    unsigned diffIndex(const KDPoint &P, const KDPoint &Q, unsigned s = 0) {
        for (unsigned i = 0; i < m_dimension; ++i, ++s %= m_dimension)
            if (P.first[s] != Q.first[s])
                return s;

        return m_dimension;
    }

    unsigned diffIndex(const KDPlane &plane, unsigned s = 0) {
        return diffIndex(plane.first, plane.second, s);
    }

protected:

    class KDNode {
    public:
        virtual bool        isInternal() const = 0;
        virtual KDPlane     boundedPlane() const = 0;
        virtual void        nearestNeighbor(const KDPoint &srcPoint, 
                                    KDPoint &nearPoint,
                                    T &minDistance, 
                                    KDPlane &minPlane) const = 0;
    };

    class KDInnerNode : public KDNode
    {
    private:
        unsigned    m_axis;
        T           m_splitVal;

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

        virtual void nearestNeighbor(const KDPoint &srcPoint,
                                    KDPoint &nearPoint,
                                    T &minDistance,
                                    KDPlane &minPlane) 
                                    const override {

            if (intersectPlane(m_left->boundedPlane(), minPlane))
                m_left->nearestNeighbor(srcPoint, nearPoint, minDistance, minPlane);

            if (intersectPlane(m_right->boundedPlane(), minPlane))
                m_right->nearestNeighbor(srcPoint, nearPoint, minDistance, minPlane);
        }
    };

    class KDLeafNode : public KDNode {

    private:
        KDPoint m_pointCoords;

    public:
        WKPTR<KDLeafNode> m_prev, m_next;

        KDLeafNode(const KDPoint &point) : m_pointCoords(point) { }

        KDLeafNode(const KDPoint &point, WKPTR<KDLeafNode> Prev, 
                WKPTR<KDLeafNode> Next) :
                    m_pointCoords(point),
                    m_prev(Prev),
                    m_next(Next) {}

        const KDPoint pointCoords() const {
            return m_pointCoords;
        }

        virtual bool isInternal() const override  {
            return false;
        }

        virtual KDPlane boundedPlane() const override  {
            return { m_pointCoords, m_pointCoords };
        }

        virtual void nearestNeighbor(const KDPoint &srcPoint,
                                     KDPoint &nearPoint,
                                     T &minDistance, 
                                     KDPlane &minPlane) const override {
            
            T currDistance = getDistance(srcPoint, m_pointCoords);

            if (currDistance < minDistance) {
                nearPoint   = m_pointCoords;
                minDistance = currDistance;
                minPlane   = createPlane(srcPoint, minDistance);
            }
        }
    };

    T getAxisMedian(KDPointIterator iterBegin,
                         KDPointIterator iterEnd,
                         const unsigned num) {
        
        std::sort(iterBegin, iterEnd, [num](const KDPoint &A, const KDPoint &B) {
                return A.first[num] < B.first[num];
        });

        size_t numPoints = iterEnd - iterBegin;

        T median = (*(iterBegin + numPoints / 2)).first[num];

        if (numPoints % 2 == 0)
            median = (median + (*(iterBegin + numPoints / 2 - 1)).first[num]) / 2.0f;

        if (median == (*(iterEnd - 1)).first[num] && median != (*iterBegin).first[num]) {
            KDPointIterator medianIter = iterBegin + numPoints / 2;

            while (median == (*(--medianIter)).first[num]);

            median = (median + (*medianIter).first[num]) / 2.0f;
        }

        return median;
    }

    SHRPTR<KDNode> createKDTree(KDPointIterator iterBegin,
                                KDPointIterator iterEnd,
                                SHRPTR<KDLeafNode> &lastLeaf,
                                unsigned Depth = 0) {
        
        if (iterEnd - iterBegin == 1) {
            SHRPTR<KDLeafNode> retNode(new KDLeafNode(*iterBegin));

            if (lastLeaf) {
                lastLeaf->m_next = retNode;
                retNode->m_prev = lastLeaf;
            } else
                m_firstLeaf = retNode;

            lastLeaf = retNode;

            return retNode;
        }
        else if (iterEnd - iterBegin == 2) {
            KDPoint point0 = *iterBegin, point1 = *(std::next(iterBegin));
            unsigned splitAxis = diffIndex(point0, point1, Depth % m_dimension);

            if (point0.first[splitAxis] > point1.first[splitAxis])
                std::swap(point0, point1);

            SHRPTR<KDLeafNode> Left(new KDLeafNode(point0)), 
                Right(new KDLeafNode(point1));

            if (lastLeaf) {
                lastLeaf->m_next = Left;
                Left->m_prev = lastLeaf;
            } else
                m_firstLeaf = Left;

            Left->m_next = Right;
            Right->m_prev = Left;

            lastLeaf = Right;

            SHRPTR<KDInnerNode> retNode(new KDInnerNode((point0.first[splitAxis] + 
                            point1.first[splitAxis]) / 2.0f, splitAxis,
                            createPlane(point0, point1), Left, Right));

            return retNode;
        } else {
            KDPlane     boundedPlane = createPlane(iterBegin, iterEnd);
            unsigned    splitAxis    = diffIndex(boundedPlane, Depth%m_dimension);
            T           median       = getAxisMedian(iterBegin, iterEnd, splitAxis);
            size_t      numPoints    = iterEnd - iterBegin;

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

    KDPoint semiNearestNeighborPoint(const KDPoint &srcPoint) const {
        SHRPTR<KDNode> node(m_root);
        SHRPTR<KDInnerNode> iNode;

        while (node->isInternal()) {
            iNode = std::static_pointer_cast<KDInnerNode>(node);

            node = (srcPoint.first[iNode->splitAxis()] <= iNode->splitVal()) ? 
                iNode->m_left : iNode->m_right;
        }

        SHRPTR<KDLeafNode> retval = std::static_pointer_cast<KDLeafNode>(node);

        return retval->pointCoords();
    }

    SHRPTR<KDNode>          m_root;
    WKPTR<KDLeafNode>       m_firstLeaf;

public:

    KDTree(int dim) {
        m_dimension = dim;
    }       
    
    KDTree(std::vector<KDPoint> &Points, int dim) {
        m_dimension = dim;
        insert(Points);
    }

    void clear() {
        m_root.reset();
    }

    ~KDTree() {
        m_root.reset();
    }

    KDPlane boundedPlane() const {
        if (m_root)
            return m_root->boundedPlane();
        else {
            KDPoint P1, P2;

            for (int i = 0; i < m_dimension; i++){
                P1.first.push_back(std::numeric_limits<T>::quiet_NaN());
                P2.first.push_back(std::numeric_limits<T>::quiet_NaN());
            }

            return KDPlane(P1, P2);
        }
    }

    void insert(std::vector<KDPoint> &Points) {
        this->clear();

        if (Points.size() > 0) {

            sort(Points.begin(), Points.end());
            
            KDPointIterator it =  unique(Points.begin(), Points.end());
            
            Points.resize(distance(Points.begin(), it));
            Points.shrink_to_fit();

            SHRPTR<KDLeafNode> dummy;

            m_root = createKDTree(Points.begin(), Points.end(), dummy);
        }
    }

    void rebuild(std::vector<KDPoint> &points) {
        insert(points);
    }

    bool nearestNeighbor(const KDPoint &srcPoint, KDPoint &nearPoint) const {
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

    void printKDTree(SHRPTR<KDNode> node, 
                    std::ofstream &dumpFile,
                    unsigned depth = 0) {
        if (node == NULL)
            dumpFile << "NULL" << std::endl;
        else {
            if (node->isInternal()) {
                SHRPTR<KDInnerNode> iNode = std::static_pointer_cast
                    <KDInnerNode>(node);

                printKDTree(iNode->m_left, dumpFile, depth + 1);
                printKDTree(iNode->m_right, dumpFile, depth + 1);
            }
            else {
                SHRPTR<KDLeafNode> lNode = std::static_pointer_cast
                    <KDLeafNode>(node);

                KDPoint point = lNode->pointCoords();

                std::stringstream ss;

                ss << point.second << " ";

                for (unsigned i = 0; i < m_dimension; i++)
                    ss << PRINTPRECISE << point.first[i] << " ";

                dumpFile << ss.str() << std::endl;;
            }
        }
    }

    void printKDTree(const std::string& outputFile) {
        std::ofstream           dumpFile;

        dumpFile.open (outputFile.c_str(),  std::ofstream::out);
        
        printKDTree(m_root, dumpFile, 0);
    }
};




