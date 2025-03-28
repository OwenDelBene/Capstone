#include "KDTree.h"


int main() {
    // Create a KDTree with 2 dimensions
    KDTree<2, 3> tree;
    
    // Insert points into the KDTree
    /*
    tree.insert({3, 6}, {1, 2, 3});
    tree.insert({2, 2}, {4, 5, 6});
    tree.insert({4, 7}, {7, 8, 9});
    tree.insert({1, 3}, {10, 11, 12});
    tree.insert({2, 4}, {13, 14, 15});
    tree.insert({5, 4}, {16, 17, 18});
    tree.insert({7, 2}, {19, 20, 21});
    */
    fillTree<2, 3>(&tree, "./test.db");
    // Print the KDTree structure
    cout << "KD Tree structure:" << endl;
    tree.print();

    // Search for specific points in the KDTree
    array<Angle_Type, 2> searchPoint = {1, 2};
    Pwm_Type out[3] = {0, 0, 0};
    cout << "\nSearching for point (1, 2): " 
         << (tree.search(searchPoint, out) ? "Found" : "Not found") << endl;
    cout << "pwm values: "<< out[0] << " " << out[1] << " " << out[2] << endl;
    searchPoint = {6, 3};
    cout << "Searching for point (6, 3): " 
         << (tree.search(searchPoint, out) ? "Found" : "Not found") << endl;

    return 0;
}
