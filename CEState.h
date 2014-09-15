/* 
 * File:   CEState.h
 * Author: jingru
 *
 * Created on February 19, 2012, 10:48 AM
 */

#ifndef CESTATE_H
#define	CESTATE_H
#include<vector>
#include <iostream>
#include <assert.h>
using namespace std;

/**
 * configuration representation
 */
class CEState
{
public:
    CEState(){}
    CEState(int n){
        q.resize(n);
    }

    CEState(const CEState &orig){
        this->q = orig.q;
    }
    ~CEState(void){
        q.clear();
    }
    
    int size(){
        return q.size();
    }
    
    void print(){
        if(q.size() == 0)
            return;
        int numD = q.size();
        for(int i = 0; i < numD; i++){
            cout<<q[i]<<" ";
        }
        cout<<endl;
    }

    void print(std::ostream &out){
        if(q.size() == 0)
            return;
        int numD = q.size();
        for(int i = 0; i < numD; i++){
        	out<<q[i]<<" ";
        }
        out<<endl;
    }

    void setZero(){
        if(q.size() == 0)
            return;
        for(int i = 0; i < q.size();i++){
            q[i] = 0;
        }
    }
    
    void setValue(double val){
        if(q.size() == 0)
            return;
        for(int i = 0; i < q.size();i++){
            q[i] = val;
        }
    }
    
    
    double operator [](int i) const{
        return q[i];
    }
    /** configuration */
    vector<double> q;
};

inline vector<double> operator - (const vector<double>& a, const vector<double>& b)
{
    assert(a.size() == b.size());
    vector<double> c;
    for(int i = 0; i < a.size(); i++)
    {
        c.push_back(b[i]-a[i]);
    }
    return c;
}
#endif	/* CESTATE_H */

