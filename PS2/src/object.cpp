#include "object.hpp"

#include "helpers.hpp"

#include <fstream>
#include <sstream>


Object::Object(string filename) {
    Vertex null_v{0, 0, 0};
    Normal null_vn{0, 0, 0};
    vertices.push_back(null_v);
    normals.push_back(null_vn);

    ifstream infile(filename);

    string t;
    while (infile >> t) {
        if (t == "v") {
            double x, y, z;
            infile >> x >> y >> z;
            Vertex v{x, y, z};
            vertices.push_back(v);
        }
        else if (t == "vn") {
            double x, y, z;
            infile >> x >> y >> z;
            Normal vn{x, y, z};
            normals.push_back(vn);
        }
        else if (t == "f") {
            string s1, s2, s3;
            infile >> s1 >> s2 >> s3;
            int v1, v2, v3, n1, n2, n3;

            tokenize_string(s1, "//") >> v1 >> n1;
            tokenize_string(s2, "//") >> v2 >> n2;
            tokenize_string(s3, "//") >> v3 >> n3;

            Face f{v1, v2, v3, n1, n2, n3};
            faces.push_back(f);
        }
    }
}
