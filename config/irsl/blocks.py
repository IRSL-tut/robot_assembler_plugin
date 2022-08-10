import sys

stud = 8.0

def print_connecting_point(name, _type, x, y, z, rotate = None):
    print('      -')
    print('        name: {}'.format(name))
    print('        types: [{}]'.format(_type))
    print('        translation: [ {}, {}, {}]'.format(x, y, z))
    if not (rotate is None):
        print('        rotation: [ 0, 0, 1, {}]'.format(rotate))

def print_plate(row = 1, column = 1, color = None, strm = sys.stdout):
    height_ = (2 * stud)/5
    print_one('plate_{}_{}'.format(row,column), row, column, height_, color, strm)

def print_block(row = 1, column = 1, height = 1, color = None, strm = sys.stdout):
    height_ = (6 * height * stud)/5
    if height == 1:
        name = 'block_{}_{}'.format(row,column)
    else:
        name = 'block_{}_{}_{}'.format(row,column,height)
    print_one(name, row, column, height_, color, strm)

def print_one(name, row, column, height_, color = None, strm = sys.stdout):
    print('  -')
    print('    type: {}'.format(name))
    print_one_geom(row, column, height_, color, strm)

def print_one_geom(row, column, height_, color = None, strm = sys.stdout):
    ## box [ row * stud, colum * stud, (2 * stud)/5 ]
    row_  = row * stud
    column_ = column * stud
    print('    visual:')
    print('      -')
    print('        translation: [ {}, {}, {} ]'.format(row_/2, column_/2, -height_/2))
    print('        box: [ {}, {}, {} ]'.format(row_, column_, height_))
    print('    collision:')
    print('      -')
    print('        translation: [ {}, {}, {} ]'.format(row_/2, column_/2, -height_/2))
    print('        box: [ {}, {}, {} ]'.format(row_, column_, height_))
    print('    mass-param:')
    print('      mass: {}'.format(1.0)) ## dummy
    print('      center-of-mass: [{}, {}, {}]'.format(row_/2, column_/2, -height_/2))
    print('      inertia-tensor: [{}, 0.0, 0.0,  0.0, {}, 0.0,  0.0, 0.0, {}]'.format(1, 1, 1)) ## dummy
    print('    connecting-points:')
    for ix in range(0, row):
        for iy in range(0, column):
            print_connecting_point('PL1_{}_{}'.format(ix,iy), 'PC1', (ix + 0.5) * stud, (iy + 0.5) * stud, 0)
            print_connecting_point('SK1_{}_{}'.format(ix,iy), 'SC1', (ix + 0.5) * stud, (iy + 0.5) * stud, - height_)
            if iy > 0:
                print_connecting_point('PL2X_{}_{}'.format(ix,iy), 'PC2', (ix + 0.5) * stud, (iy + 0.0) * stud, 0, rotate = 90)
                print_connecting_point('SK2X_{}_{}'.format(ix,iy), 'SC2', (ix + 0.5) * stud, (iy + 0.0) * stud, - height_, rotate = 90)
            if ix > 0:
                print_connecting_point('PL2Y_{}_{}'.format(ix,iy), 'PC2', (ix + 0.0) * stud, (iy + 0.5) * stud, 0)
                print_connecting_point('SK2Y_{}_{}'.format(ix,iy), 'SC2', (ix + 0.0) * stud, (iy + 0.5) * stud, - height_)
            if ix > 0 and iy > 0:
                print_connecting_point('PL4_{}_{}'.format(ix,iy), 'PC4', ix * stud, iy * stud, 0)
                print_connecting_point('SK4_{}_{}'.format(ix,iy), 'SC4', ix * stud, iy * stud, - height_)

plate_lst = [
(1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6), (1, 8), (1, 10), (1, 12),
        (2, 2), (2, 3), (2, 4),         (2, 6), (2, 8), (2 ,10), (2, 12), (2, 14), (2, 16),
                (3, 3),
                        (4, 4),         (4, 6), (4, 8), (4, 10), (4, 12),
                                        (6, 6), (6, 8), (6, 10), (6, 12), (6, 14), (6, 16), (6, 24),
                                                (8, 8),                            (8, 16),
                                                                                   (16, 16),
    ]

for row,col in plate_lst:
    print_plate(row, col)

block_lst = [
(1, 1, 1), (1, 2, 1), (1, 3, 1), (1, 4, 1), (1, 6, 1), (1, 8, 1), (1, 10, 1), (1, 12, 1), (1, 16, 1),
           (2, 2, 1), (1, 3, 1), (1, 4, 1), (1, 6, 1), (1, 8, 1), (1, 10, 1),
(1, 1, 3), (1, 1, 5),
(1, 2, 2), (1, 2, 5),
(1, 3, 5), (1, 4, 3),
(2, 2, 3),
]

for row,col,height in block_lst:
    print_block(row, col, height)
