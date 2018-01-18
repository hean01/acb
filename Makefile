all: src
.PHONY: src doc
src:
	make -C src default

doc:
	cat doc/header.html > doc/readme.html
	markdown -f toc,smarty,links,idanchor README.md >> doc/readme.html
	cat doc/footer.html >> doc/readme.html
clean:
	make -C src clean
