#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();

        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
       
        int budgetNum = 4;
        float offset = centroidBounds.Diagonal()[dim] / budgetNum;

        std::vector<Bounds3> nBudget(budgetNum);
        std::vector<int> count;
        int idx = 0;
        int budgetIdx = 1;
        int num = 0;
        while (idx < objects.size()) {
            if ((objects[idx]->getBounds().Centroid() - centroidBounds.pMin)[dim] < offset * budgetIdx) {
                nBudget[budgetIdx - 1] = Union(nBudget[budgetIdx - 1], objects[idx]->getBounds());
                idx++;
                num++;
            }
            else {
                count.push_back(num);
                num = 0;
                budgetIdx++;
                if (budgetIdx == budgetNum) break;
            }
        }
        count.push_back(objects.size() - idx);
        while (idx < objects.size()) {
            nBudget[budgetIdx - 1] = Union(nBudget[budgetIdx - 1], objects[idx]->getBounds());
            idx++;
        }
        float cost = std::numeric_limits<float>::infinity();
        int countIdx = 0;
        for (int i = 0; i < nBudget.size()-1; i++) {
            Bounds3 boundLeft, boundRight;
            int countLeft = 0;
            for (int left = 0; left <= i; left++) {
                boundLeft = Union(boundLeft, nBudget[left]);
                countLeft += count[left];
            }
            for (int right = i + 1; right < nBudget.size(); right++) {
                boundRight = Union(boundLeft, nBudget[right]);
            }
            float newCost = boundLeft.SurfaceArea() * countLeft + boundRight.SurfaceArea() * (objects.size() - countLeft);
            if(newCost < cost){
                cost = newCost;
                countIdx = countLeft;
            }
        }

        auto beginning = objects.begin();
        //auto middling = objects.begin() + (objects.size() / 2);
        auto middling = objects.begin() + countIdx;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Vector3f dir = ray.direction;
    std::array<int, 3> dirIsNeg = { int(dir.x > 0), int(dir.y > 0), int(dir.z > 0) };
    Intersection isect;
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
        return isect;
    }
    if (node->object != nullptr) {
        return node->object->getIntersection(ray);
    }
    Intersection isectLeft = getIntersection(node->left, ray);
    Intersection isectRight = getIntersection(node->right, ray);
    if (isectLeft.happened) isect = isectLeft;
    if (isectRight.happened && isectRight.distance < isect.distance) isect = isectRight;
    return isect;
}