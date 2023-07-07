#include<stdlib.h>
#include<iostream>
#include<unordered_map>
#include<cmath>

using namespace std;

double max(double a, double b){
    if (b <= a) return a;
    return b;
}

unsigned long long reverse_bits(unsigned long long n) {
    short bits = 64;
    unsigned long long mask = ~(0llu); // equivalent to uint32_t mask = 0b11111111111111111111111111111111;

    while (bits >>= 1) {
        mask ^= mask << (bits); // will convert mask to 0b00000000000000001111111111111111;
        n = (n & ~mask) >> bits | (n & mask) << bits; // divide and conquer
    }

    return n;
}

class opt_set{
public:
    unsigned long long i;
    int n;
    bool b;
    opt_set(int n){
        this->n = n;
        i = 0;
        b = 0;
    }
    void set(int k){
        i |= (1llu << k);
    }
    void reset(int k){
        i &= ~(1llu << k);
    }
    void rotate(int k){
        i = (i >> k) | ((i << (n - k)) & ((1llu << n) - 1llu));
    }
    unsigned long long mirror_unvariant(){ //beg = 0 then returns a mirror unvariant version of i
        unsigned long long i2 = (reverse_bits(i-1) >> (63 - n)) + 1;
        if (i < i2) return i;
        return i2;
    }
    int log(){
        return __builtin_ctzll(i);
    }
    int count(){
        return __builtin_popcountll(i);
    }
    int operator[](int k){
        return (i >> k)%2;
    }
    bool operator==(const opt_set& el) const {
        if (el.b != b) return false;
        unsigned long long i2 = (reverse_bits(i-1) >> (63 - n)) + 1;
        return (i2 == el.i) or (i == el.i);
        //return i == el.i and (el.b == b);
    }
    unsigned long long hash(){
        return i + b * 34029489980;
    }
};

struct hashopt_set{
    unsigned long long operator()(const opt_set& el) const {
        unsigned long long i2 = (reverse_bits(el.i-1) >> (63 - el.n)) + 1;
        if (el.i < i2) return el.i + el.b * 3020849320;
        return i2 + el.b * 3020849320;
        //return el.i + (unsigned long long)el.b * 3042973024;
    }
};

double pre_bruteforce_opt(int n, opt_set& ens, unordered_map<opt_set, double, hashopt_set>& cache, double* chord_cache){
    int count = ens.count();
    if (count == 1){
        return 0;
    }
    if (count == 2){
        int temp;
        ens.i -= 1;
        temp = ens.log();
        ens.i += 1;
        return chord_cache[temp];
    }

    if (const auto& it = cache.find(ens); it != cache.end()){
        return it->second;
    }

    double res = INFINITY;
    double v;

    if (!ens.b){
        int beg2 = 0;
        int temp;
        opt_set X(n);
        X.b = true;
        X.i = ens.i;
        X.i -= 1;
        temp = X.log();
        beg2 += temp;
        X.rotate(temp);
        while(beg2 < n){
            v = pre_bruteforce_opt(n, X, cache, chord_cache);
            v += chord_cache[beg2];
            if (v < res) res = v;
            X.i -= 1;
            temp = X.log();
            X.i += 1;
            X.rotate(temp);
            beg2 += temp;
        }
        cache[ens] = res;
        return res;
    }
    //now we have 2 robots

    double res1, res2;
    int c;
    opt_set X(n);
    opt_set Xb(n);
    X.b = false;
    Xb.b = false;

    for (int i = 0; i < (1llu << (count-1)); i++){

        c = 0;
        X.i = 1;
        Xb.i = 1;
        for (int j = 1; j < n; j++){
            if (ens[j] == 1){
                if ((i >> c)%2 == 1){
                    X.set(j);
                } else {
                    Xb.set(j);
                }
                c++;
            }
        }

        res1 = pre_bruteforce_opt(n, X, cache, chord_cache);
        if (res1 >= res) continue;
        res2 = pre_bruteforce_opt(n, Xb, cache, chord_cache);
        if (max(res1, res2) < res) res = max(res1, res2);
    }
    cache[ens] = res;
    return res;
}

double bruteforce_opt(int n){
    unordered_map<opt_set, double, hashopt_set> cache;
    double* chord_cache = (double*)malloc(sizeof(double)*n);
    for (int i = 0; i < n; i++){
        chord_cache[i] = 2*sin(M_PI*i/n);
    }
    opt_set ens(n);
    ens.i = (1llu << (n)) - 1;
    ens.b = true; //two robots
    double res = pre_bruteforce_opt(n, ens, cache, chord_cache);
    free(chord_cache);
    return res+1;
}

int main(){
    cout << bruteforce_opt(20) << endl;
    return 0;
}
