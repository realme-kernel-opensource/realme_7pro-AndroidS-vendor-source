#ifndef __COMP_H__
#define __COMP_H__
#if defined(__x86_64__) || defined(__x86__) || defined(__amd64__) || defined(YACC_TEST) /* !ANDROID build */
#define ERROR(...) fprintf(stderr, __VA_ARGS__)
#define DEBUG(...) fprintf(stderr, __VA_ARGS__)
#else
#include "OIface.h"
#define ANDROID_RUNTIME
#endif
#include <string>
#include <map>
#include <vector>
typedef enum { typeCon, typeId, typeOpr, typePre1, typePre2 } nodeEnum;

/* constants */
struct conNodeType {
    int value;                  /* value of constant */
};

/* identifiers */
struct idNodeType {
    char name[256];                      /* subscript to sym array */
};

struct nodeType;
/* operators */
struct oprNodeType {
    int oper;                   /* operator */
    int nops;                   /* number of operands */
    nodeType *op[1];            /* operands, extended at runtime */
};

struct nodeType {
    nodeEnum type;              /* type of node */

    union {
        conNodeType con;        /* constants */
        idNodeType id;          /* identifiers */
        oprNodeType opr;        /* operators */
    };
};

#define compileProgram(args...)     -1
#define runProgram(args...)         -1
#define destroyProgram(x)           -1

// int compileProgram(const char* expr, std::vector<nodeType*> *syntaxTree) { return 0; }
// int runProgram(const char* clientName, const std::vector<nodeType*> &syntaxTree, const std::map<std::string, int> &pre1,
//        const std::map<std::string, int> &pre2, std::map<std::string, int> *globalVariable) { return 0; }
// int destroyProgram(const std::vector<nodeType*> &syntaxTree) { return 0; }

#endif
