// C++ Program to Implement KD Tree
#include <ios>
#include <iostream>
#include <array>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
using std::cout, std::endl, std::array, std::string;

typedef int8_t Angle_Type ;
typedef float Pwm_Type ;


// Template class for KDTree with K dimensions
template <size_t num_angles, size_t num_servo>
class KDTree {
private:
    // Node structure representing each point in the KDTree
    struct Node {
        // Point in K dimensions
        array<Angle_Type, num_angles> point; 
        // Pointer to left child
        Node* left;          
        // Pointer to right child
        Node* right;            

        array<Pwm_Type, num_servo> pwm; 
        // Constructor to initialize a Node
        Node(const array<Angle_Type, num_angles>& pt, const array<Pwm_Type, num_servo>& sv) : point(pt), pwm(sv),  left(nullptr), right(nullptr) {}
    };

    Node* root; // Root of the KDTree

    // Recursive function to insert a point into the KDTree
    Node* insertRecursive(Node* node, const array<Angle_Type, num_angles>& point, const array<Pwm_Type, num_servo>& pwm,  int depth) {
        // Base case: If node is null, create a new node
        if (node == nullptr) return new Node(point, pwm);

        // Calculate current dimension (cd)
        int cd = depth % num_angles;

        // Compare point with current node and decide to go left or right
        if (point[cd] < node->point[cd])
            node->left = insertRecursive(node->left, point, pwm, depth + 1);
        else
            node->right = insertRecursive(node->right, point,pwm,  depth + 1);

        return node;
    }

    // Recursive function to search for a point in the KDTree
    bool searchRecursive(Node* node, const array<Angle_Type, num_angles>& point, int depth, Pwm_Type out[num_servo]) const {
        // Base case: If node is null, the point is not found
        if (node == nullptr) return false;

        // If the current node matches the point, return true
        if (node->point == point) {
          for (int i=0; i< num_servo; i++ ){
            out[i] = node->pwm[i];
          }
          return true;
        }
        // Calculate current dimension (cd)
        int cd = depth % num_angles;

        // Compare point with current node and decide to go left or right
        if (point[cd] < node->point[cd])
            return searchRecursive(node->left, point, depth + 1, out);
        else
            return searchRecursive(node->right, point, depth + 1, out);
    }

    // Recursive function to print the KDTree
    void printRecursive(Node* node, int depth) const {
        // Base case: If node is null, return
        if (node == nullptr) return;

        // Print current node with indentation based on depth
        for (int i = 0; i < depth; i++) cout << "  ";
        cout << "(";
        for (size_t i = 0; i < num_angles; i++) {
            cout << +node->point[i];
            if (i < num_angles - 1) cout << ", ";
        }
        cout << ")" << endl;

        // Recursively print left and right children
        printRecursive(node->left, depth + 1);
        printRecursive(node->right, depth + 1);
    }

public:
    // Constructor to initialize the KDTree with a null root
    void insert(const array<Angle_Type, num_angles>& angles, const array<Pwm_Type, num_servo>& pwm) {
        root = insertRecursive(root, angles ,pwm,  0);
    }

    // Public function to search for a point in the KDTree
    bool search(const array<Angle_Type, num_angles>& point, Pwm_Type out[num_servo]) const {
        return searchRecursive(root, point, 0, out);
    }

    // Public function to print the KDTree
    void print() const {
        printRecursive(root, 0);
    }
};


template <size_t A, size_t B>
void fillTree(KDTree<A, B>* tree, const string& filepath) {
  std::fstream file; //(filepath);
  file.open(filepath, std::ios_base::in);
  if (! file.is_open() ) {
    //cout << "cannot open file " << endl;
    perror("Error opening file ");
    cout << filepath << endl;
  }
  string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    array<Angle_Type, A> angle;
    array<Pwm_Type, B> pwm;
    string entry;
    for (int i=0; i<A; i++) {
      std::getline(ss, entry, ',');
      angle[i] = (Angle_Type)std::stod(entry);
    }
    for (int i=0; i<B; i++) {
      std::getline(ss, entry, ',');
      pwm[i] = (Pwm_Type)std::stod(entry);
    }

    tree->insert(angle, pwm);

  }


}

