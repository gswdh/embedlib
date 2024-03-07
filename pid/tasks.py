from invoke import task

@task
def make_test(c):
    c.run("gcc -o build/test test.c pid.c")

@task
def run_test(c):
    c.run("./build/test")

