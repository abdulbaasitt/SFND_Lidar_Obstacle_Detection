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

	
	/**
	 * @brief Inserts a new point into the KD-Tree
	 * 
	 * @param node Pointer to the current node being processed
	 * @param depth Current depth of the tree
	 * @param point Point to be inserted into the tree
	 * @param id ID of the point to be inserted
	 */
	void insertHelper(Node** node, uint depth, std::vector<float>point, int id) {
		//  Tree is empty
		if (!(*node)) 
			*node = new Node(point, id); 
		else {

			//  Calculate current dim
			uint currDim = depth % 2; 
			if (point[currDim] < (*node)->point[currDim]) 
				insertHelper(&((*node)->left), depth + 1, point, id);
			else
				insertHelper(&((*node)->right), depth + 1, point, id);
		}
	}

	// function to search a point in the tree
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		//  call insertHelper function
		insertHelper(&root, 0, point, id);

	}

	/**
	 * @brief Helper function to search for points within a given distance tolerance from a target point in a KD-Tree.
	 * 
	 * @param target The target point to search around.
	 * @param root The root node of the KD-Tree.
	 * @param distanceTol The distance tolerance within which to search for points.
	 * @param depth The current depth of the KD-Tree.
	 * @param ids A vector to store the IDs of the points found within the distance tolerance.
	 */
	void searchHelper(const std::vector<float>& target, Node* root, float distanceTol, uint depth, std::vector<int>& ids) {
		//  Tree is empty
		if (!root) return;
		float left_dist_x = target[0] - distanceTol - root->point[0];
		float right_dist_x = target[0] + distanceTol - root->point[0];		
		float left_dist_y = target[1] - distanceTol - root->point[1];
		float right_dist_y = target[1] + distanceTol - root->point[1];
		//  Check if the point is in the box
		if (left_dist_x <= 0 && right_dist_x >= 0 && left_dist_y <= 0 && right_dist_y >= 0) {
			float distance = sqrt(pow(target[0] - root->point[0], 2) + pow(target[1] - root->point[1], 2));
			if (distance <= distanceTol)
				ids.push_back(root->id);
		}

		//  Check across boundary
		if (target[depth % 2] - distanceTol < root->point[depth % 2])
			searchHelper(target, root->left, distanceTol, depth + 1, ids);
		if (target[depth % 2] + distanceTol > root->point[depth % 2])
			searchHelper(target, root->right, distanceTol, depth + 1, ids);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		std::cout << "target: " << target[0] << "," << target[1] << "DistanceTol: " << distanceTol << std::endl;
		//  call searchHelper function
		searchHelper(target, root, distanceTol, 0, ids);
		std::cout << "Neighbour Point Size: " << ids.size() << std::endl;
		return ids;
	}
	

};




