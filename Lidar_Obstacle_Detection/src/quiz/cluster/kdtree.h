/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	
        void insertHelper(Node** node,uint dep,std::vector<float> point,int id)
         {
          if (*node==NULL)
             {
                *node=new Node(point,id);
             }
         
          else
             {
               uint cd = dep%2;
                        
                    if (point[cd] < ((*node)->point[cd]))
                      {
                       insertHelper(&((*node)->left),dep+1, point,id);   
                      }
                    else
                      {
                       insertHelper(&((*node)->right),dep+1, point,id);   
                      }

             }
         }
  
        void insert(std::vector<float> point, int id)
	{
          insertHelper(&root,0,point,id);
	}


        void searchHelper(std::vector<float> target,Node* node,float distanceTol,std::vector<int>& ids,uint dep)
         {
           
           if (node!=NULL)
            {
           if ((node->point[0]>=(target[0]-distanceTol))&&(node->point[0]<=(target[0]+distanceTol))&&(node->point[1]>=(target[1]-distanceTol))&&(node->point[1]<=(target[1]+distanceTol)))
              {
                std::cout << "Test Search_2" << std::endl;
                float distancej = sqrt(((node->point[0]-target[0])*(node->point[0]-target[0]))+((node->point[1]-target[1])*(node->point[1]-target[1])));
                if (distancej<=distanceTol)
                    {
                    ids.push_back(node->id);
                    }
              }

              std::cout << "Test Search_3" << std::endl;
               if((target[dep%2]-distanceTol)< (node->point[dep%2]))
                    searchHelper(target,node->left,distanceTol,ids,dep+1);  
               if((target[dep%2]-distanceTol)> (node->point[dep%2]))
                    searchHelper(target,node->right,distanceTol,ids,dep+1); 
           }
         }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
                std::cout << "Test Search_1" << std::endl;

                searchHelper(target,root,distanceTol,ids,0);
               
		return ids;
	}
	

};




