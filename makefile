# Makefile para el proyecto CVRP Solver

CXX = g++
CXXFLAGS = -std=c++17 -Wall -g -I.
# Banderas de enlace para COIN-OR
LIBS = -lClp -lCoinUtils 

# Target por defecto
all: test_parser test_route test_greedy test_kopt test_bb

# --- Ejecutables de Prueba ---

test_parser: test_Parser.cpp Parser.o Client.o
	$(CXX) $(CXXFLAGS) test_Parser.cpp Parser.o Client.o -o test_parser

test_route: test_Route.cpp Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) test_Route.cpp Route.o Parser.o Client.o -o test_route

test_greedy: test_Greedy.cpp GreedyBuilder.o Solution.o Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) test_Greedy.cpp GreedyBuilder.o Solution.o Route.o Parser.o Client.o -o test_greedy

test_kopt: test_KOpt.cpp KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) test_KOpt.cpp KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o -o test_kopt

# NUEVO: Ejecutable para Branch & Bound
test_bb: test_BB.cpp BranchAndBound.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) test_BB.cpp BranchAndBound.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o -o test_bb $(LIBS)

test_cbc: test_cbc.cpp CbcSolver.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o KOpt.o
	$(CXX) $(CXXFLAGS) test_cbc.cpp CbcSolver.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o KOpt.o -o test_cbc -lCbc -lCgl -lOsiClp -lOsi -lClp -lCoinUtils
# --- Reglas para Compilar Objetos (.o) ---

Client.o: Client.cpp Client.h
	$(CXX) $(CXXFLAGS) -c Client.cpp -o Client.o

Parser.o: Parser.cpp Parser.h Client.h
	$(CXX) $(CXXFLAGS) -c Parser.cpp -o Parser.o

Route.o: Route.cpp Route.h Parser.h
	$(CXX) $(CXXFLAGS) -c Route.cpp -o Route.o

Solution.o: Solution.cpp Solution.h Route.h Parser.h
	$(CXX) $(CXXFLAGS) -c Solution.cpp -o Solution.o

GreedyBuilder.o: GreedyBuilder.cpp GreedyBuilder.h Solution.h Parser.h
	$(CXX) $(CXXFLAGS) -c GreedyBuilder.cpp -o GreedyBuilder.o

KOpt.o: KOpt.cpp KOpt.h Solution.h Parser.h Route.h
	$(CXX) $(CXXFLAGS) -c KOpt.cpp -o KOpt.o

# NUEVO: Objeto de Branch and Bound
BranchAndBound.o: BranchAndBound.cpp BranchAndBound.h Solution.h Parser.h KOpt.h
	$(CXX) $(CXXFLAGS) -c BranchAndBound.cpp -o BranchAndBound.o

CbcSolver.o: CbcSolver.cpp CbcSolver.h Parser.h Solution.h
	$(CXX) $(CXXFLAGS) -c CbcSolver.cpp -o CbcSolver.o

# --- Utilidades ---

clean:
	rm -f *.o test_parser test_route test_greedy test_kopt test_bb test_cbc