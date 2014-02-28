EXEC=rgb

$(EXEC): rgb.rc *.rs
	rustc $<

clean:
	rm -f $(EXEC)
