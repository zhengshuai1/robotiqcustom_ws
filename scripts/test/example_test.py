#!/usr/bin/env python

print('wo我是robot')
print(b'one' + b'two')
print('one' + 'two')

def a():
    return 1

if a() > 4:
    print('true')
else:
    print('false')
a = [1, 2, 4]
b = a[:]
assert b == a and b is not a, 'true'
a = [x for x in range(6)]
b = a[::2]
print(b)
first, second, *other = a
print(first, second, other)

class Tool:
    def __init__(self, name):
        self.name = name
    def __repr__(self):
        return f'Tool({self.name})'
a = Tool('first')
print(a.__dict__)
tools = [Tool('level'), Tool('hammer')]
print(tools)
tools.sort(key=lambda x: x.name)
print(tools)

def my_func(**kwargs):
    for key, value in kwargs.items():
        print(f'{key} = {value}')
my_func(goose='gosling', kangaroo='joey')
a = [x for x in range(6)]
print(next(iter(a)))

def my_generator():
    for i in range(9):
        yield i
def my_fun(*args):
    print(args)
it = my_generator()
my_fun(*it)

from time import sleep
from datetime import datetime
from typing import Optional

def log_typed(message: str,
    when: Optional[datetime]=None) -> None:
    """Log a message with a timestamp.

    Args:
        message: Message to print.
        when: datetime of when the message occurred.
            Defaults to the present time.
    """
    if when is None:
        when = datetime.now()
    print(f'{when}: {message}')

log_typed('Hi there!')
sleep(0.1)
log_typed('Hello again!')

def safe_division_e(numrator, denominator, /, ndigits=10, *,ignore_overflow=False, ignore_zero_division=False):
    pass

from functools import wraps
def trace(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        result = func(*args, **kwargs)
        print(f'{func.__name__}({args!r}, {kwargs!r}) '
              f'-> {result!r}')
        return result
    return wrapper

@trace
def fibonacci(n):
    """Return the n-th Fibonacci number"""
    if n in (0, 1):
        return n
    return (fibonacci(n - 2) + fibonacci(n - 1))

fibonacci(3)
help(fibonacci)
from collections.abc import Iterator
assert isinstance(iter([]), Iterator)

class Test(object):
    def __init__(self, name='test'):
        self.name = name
    def __repr__(self):
        return f'this is {self.name}'

t = Test()
print(t)

value = [100, 57, 15]
it = (x for x in value)
# print(next(it))
roots = ((x, x*2) for x in it)
print(next(roots))

class Timer:
    def __init__(self, period):
        self.current = period
        self.period = period
    def reset(self):
        self.current = self.period
    def __iter__(self):
        while self.current:
            self.current-=1
            yield self.current
def announce(text):
    print(f'{text} ticks remaining')

def run():
    timer = Timer(4)
    for current in timer:
        announce(current)
run()

# clear and nest
from collections import namedtuple, defaultdict
Grade = namedtuple('Grade', ('score', 'weight'))
user = Grade(80, 0.5)
print(user.score, user.weight, user._fields)

class Subject:
    def __init__(self):
        self.grade = []

class Student:
    def __init__(self):
        self._subjects = defaultdict(Subject)

class Gradebook:
    def __init__(self):
        self._students = defaultdict(Student)

current = {'green':12, 'blue':3}
increments = [('red', 5), ('orange', 9)]
class CountMissing:
    def __init__(self):
        self.added = 0
    def missing(self):
        self.added += 1
        return 0
    def __call__(self, *args, **kwargs):
        self.added += 1
        return 0
counter = CountMissing()
result = defaultdict(counter, current)
for key, amount in increments:
    result[key] += amount
assert counter.added == 2
print(result, counter.added)

class InputData:
    def read(self):
        raise NotImplementedError
class PathInputData(InputData):
    def __init__(self, path):
        super().__init__()
        self.path = path
    def read(self):
        with open(self.path) as f:
            return f.read()

class Test:
    def __init__(self):
        self.name = 1
        self.__name = 2
t = Test()
# print(dir(t), t.__dict__)
class MyOtherObject:
    def __init__(self):
        self.__private_field = 71
    @classmethod
    def get_private_field_of_instance(cls, instance):
        return instance.__private_field
    def __repr__(self):
        print(f'private_field is {self.__private_field}')
    def __str__(self):
        print(f'private_field is {self.__private_field}')
bar = MyOtherObject()
assert MyOtherObject.get_private_field_of_instance(bar) == 71
# bar._MyOtherObject__private_field = 80
# print(bar)

class Student:
    def __init__(self):
        self._score = 0
    @property
    def score(self):
        return self._score
    @score.setter
    def score(self, value):
        if not isinstance(value, int):
            raise ValueError('score must be integer')
        if value < 0 or value > 100:
            raise ValueError('score must between 0 -100')
        self._value = value

s = Student()
# print(s.score)
# s.score = 101

import subprocess
result = subprocess.run(
    ['echo', 'hello world'],
    capture_output=True,
    encoding='utf-8'
)
result.check_returncode()
print(result.stdout)

from threading import Thread
class Counter:
    def __init__(self):
        self.count = 0
    def increment(self, offest):
        self.count += offest
def worker(sensor_indx, num, counter):
    for _ in range(num):
        counter.increment(1)
counter = Counter()
num = 10**5
threads = []
for i in range(5):
    thread = Thread(target=worker, args=(i, num, counter))
    threads.append(thread)
    thread.start()
for thread in threads:
    thread.join()
expected = num * 5
found = counter.count
print(f'Counter should be {expected}, but got {found}')

from queue import Queue
my_queue = Queue()
def consumer():
    print('cosumer waiting')
    my_queue.get()
    print('consumer done')
thread = Thread(target=consumer)
thread.start()
print('producer putting')
my_queue.put(object())
print('producer done')
thread.join()

ALIVE = '*'
EMPTY = '-'
class Grid:
    def __init__(self, height, width):
        self.height = height
        self.width = width
        self.rows = []
        for _ in range(self.height):
            self.rows.append([EMPTY]*self.width)
    def get(self, y, x):
        return self.rows[y % self.height][x % self.width]
    def set(self, y, x, state):
        self.rows[y % self.height][x % self.width] = state
    def __str__(self):
        return f'{self.rows}'
grid = Grid(5, 9)
grid.set(0, 3, ALIVE)
grid.set(1, 4, ALIVE)
grid.set(2, 2, ALIVE)
grid.set(2, 3, ALIVE)
grid.set(2, 4, ALIVE)
# print(grid)

# raise ZeroDivisionError
# try:
#     print(1)
#     # raise ZeroDivisionError('zero')
#     raise
#     print(2)
# except ZeroDivisionError as e:
#     print('cause exception', repr(e), type(e))

def palindrome(word):
    """Return True if the given word is a palindrome."""
    return word == word[::-1]
assert palindrome('tacocat')
assert not palindrome('banana')
print(repr(palindrome.__doc__))

class Error(Exception):
    """Base-class for all exceptions raised by this module."""
class InvalidDensityError(Error):
    """There was a problem with a provided density value."""
class InvalidVolumeError(Error):
    """There was a problem with the provided weight value."""
import logging
def determine_weight(volume, density):
    if density < 0:
        raise InvalidDensityError('Density must be positive')
    if volume < 0:
        raise InvalidVolumeError('Volume must be positive')
    if volume == 0:
        density / volume

try:
    weight = determine_weight(1, -1)
except Error:
    logging.exception('Unexpected error')




