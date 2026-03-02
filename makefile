# Makefile para el proyecto CVRP Solver

CXX = g++
CXXFLAGS = -std=c++17 -Wall -g -I.

# Banderas de enlace para COIN-OR
LIBS_BASE = -lClp -lCoinUtils
LIBS_CBC  = -lCbc -lCgl -lOsiClp -lOsi -lClp -lCoinUtils
CBC_LIBS = -lCbc -lCbcSolver -lCgl -lClp -lOsi -lOsiClp -lOsiCbc -lCoinUtils

# Carpeta donde viven los tests
TESTS_DIR = tests

# Target por defecto
all: test_parser test_route test_greedy test_kopt test_vns test_bb test_bbvns test_cbc

# ---------------------------------------------------------------
# Ejecutables de Prueba
# ---------------------------------------------------------------

test_parser: $(TESTS_DIR)/test_Parser.cpp Parser.o Client.o
	$(CXX) $(CXXFLAGS) $(TESTS_DIR)/test_Parser.cpp Parser.o Client.o -o test_parser

test_route: $(TESTS_DIR)/test_Route.cpp Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) $(TESTS_DIR)/test_Route.cpp Route.o Parser.o Client.o -o test_route

test_greedy: $(TESTS_DIR)/test_Greedy.cpp GreedyBuilder.o Solution.o Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) $(TESTS_DIR)/test_Greedy.cpp GreedyBuilder.o Solution.o Route.o Parser.o Client.o -o test_greedy

test_kopt: $(TESTS_DIR)/test_KOpt.cpp KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) $(TESTS_DIR)/test_KOpt.cpp KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o -o test_kopt

test_vns: $(TESTS_DIR)/test_VNS.cpp VNS.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) $(TESTS_DIR)/test_VNS.cpp VNS.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o -o test_vns

test_bb: $(TESTS_DIR)/test_BB.cpp BranchAndBound.o VNS.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) $(TESTS_DIR)/test_BB.cpp BranchAndBound.o VNS.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o -o test_bb $(LIBS_BASE)

test_cbc: $(TESTS_DIR)/test_cbc.cpp CbcSolver.o SubtourCut.o VNS.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) $(TESTS_DIR)/test_cbc.cpp CbcSolver.o SubtourCut.o VNS.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o -o test_cbc $(LIBS_CBC)

test_bbvns: $(TESTS_DIR)/test_bbvns.cpp BranchAndBound.o VNS.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) $(TESTS_DIR)/test_bbvns.cpp BranchAndBound.o VNS.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o -o test_bbvns $(LIBS_BASE)

test_alns: tests/test_alns.cpp ALNS.o CbcSolver.o SubtourCut.o VNS.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o
	$(CXX) $(CXXFLAGS) $(TESTS_DIR)/test_alns.cpp ALNS.o CbcSolver.o SubtourCut.o VNS.o KOpt.o GreedyBuilder.o Solution.o Route.o Parser.o Client.o -o test_alns $(CBC_LIBS)
# ---------------------------------------------------------------
# Reglas para compilar objetos (.o)
# Estos siguen viviendo en la raíz del proyecto
# ---------------------------------------------------------------

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

VNS.o: VNS.cpp VNS.h KOpt.h Solution.h Parser.h Route.h
	$(CXX) $(CXXFLAGS) -c VNS.cpp -o VNS.o

BranchAndBound.o: BranchAndBound.cpp BranchAndBound.h Solution.h Parser.h KOpt.h VNS.h
	$(CXX) $(CXXFLAGS) -c BranchAndBound.cpp -o BranchAndBound.o

SubtourCut.o: SubtourCut.cpp SubtourCut.h Parser.h
	$(CXX) $(CXXFLAGS) -c SubtourCut.cpp -o SubtourCut.o

CbcSolver.o: CbcSolver.cpp CbcSolver.h Parser.h Solution.h VNS.h SubtourCut.h
	$(CXX) $(CXXFLAGS) -c CbcSolver.cpp -o CbcSolver.o

ALNS.o: ALNS.cpp ALNS.h Parser.h Solution.h VNS.h
	$(CXX) $(CXXFLAGS) -c ALNS.cpp -o ALNS.o

# ---------------------------------------------------------------
# Utilidades
# ---------------------------------------------------------------

clean:
	rm -f *.o test_parser test_route test_greedy test_kopt test_vns test_bb test_cbc test_bbvns test_alns