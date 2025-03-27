TARGET = ekf_test

CXXFLAGS = -g
srcs = *.cpp

$(TARGET):
	$(CXX) $(CXXFLAGS) $(srcs)
	./a.out
	python3 plot.py
