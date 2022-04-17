#include <iostream>
#include <vector>

using std::cout; using std::cin;
using std::endl; using std::vector;
using std::copy;

int main()
{
    vector<char> arr = {'w','x','y','z'};
    int flag;

    flag = cin.get();

    for(auto const& value: arr)
        cout << value << "; ";
    cout << "\nDone !" << endl;

    return EXIT_SUCCESS;
}