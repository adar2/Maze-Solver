//
// Created by adar on 12/5/2020.
//
#include <queue>
#include <string>
using namespace std;

struct Node{
private:
    float cost;
    int x;
    int y;
public:
    Node():cost(0),x(0),y(0){};
    Node(float cost,int x,int y):cost(cost),x(x),y(y){};
    float getCost(){return cost;};
    int getX(){return x;};
    int getY(){return y;};
};

int uniformCostSearch(int **array, int *start, int *target){
    int pathCost = 0 ;
    vector<string> path = vector<string>();
    vector<struct Node> explored = vector<struct Node>();
    priority_queue<struct Node> openList = priority_queue<struct Node>();
    struct Node sourceNode = Node(0, start[1], start[0]);
    struct Node goalNode = Node(0, target[1], target[0]);
    struct Node current_node;
    openList.push(sourceNode);
    while (!openList.empty()){
         current_node = openList.top();
         openList.pop();
         explored.push_back(current_node);
         if(current_node.getX() == goalNode.getX() && current_node.getY() == goalNode.getY())
             // reached to goal state
             return 0;
         //UP
         if(current_node.getY() - 1 >= 0 && array[current_node.getX()][current_node.getY() - 1] >=0){
             struct Node node = Node(array[current_node.getX()][current_node.getY() - 1],current_node.getX(),current_node.getY() - 1);
             openList.push(node);
         }


         //UP-RIGHT
        if(current_node.getY() - 1 >= 0 && current_node.getX() + 1 < (sizeof(array)/sizeof(int)) && array[current_node.getX()][current_node.getY() - 1] >=0){
            struct Node node = Node(array[current_node.getX()][current_node.getY() - 1],current_node.getX(),current_node.getY() - 1);
            openList.push(node);
        }
         //RIGHT

         //DOWN-RIGHT

         //DOWN

         //DOWN-LEFT

         //LEFT

         //UP-LEFT

    }
    return 1;
}
