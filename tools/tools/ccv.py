import sympy
import esym




if __name__ == '__main__':
    p1 = esym.vector('p1')
    c1 = esym.vector('c1')
    c2 = esym.vector('c2')
    n1 = esym.vector('n1')
    n2 = esym.vector('n2')

    r1 = sympy.Symbol('r1', positive=True)
    r2 = sympy.Symbol('r2', positive=True)
    p2 = esym.vector('p2')

    facts = [
        (esym.norm(c1 - p1) ** 2) - (r1 ** 2),
        (esym.norm(c2 - p1) ** 2) - (r2 ** 2),
        (p1 - c1).dot(n1),
        (p1 - c2).dot(n2),

        esym.norm(c1 - p2) - r1,
        esym.norm(c2 - p2) - r2,
        (p2 - c1).dot(n1),
        (p2 - c2).dot(n2),

    ]

    t = sympy.Symbol('t')
    print facts[0]

    facts_2 = [
        esym.norm((n1 * t) - c1) - r1,
        # esym.norm((n1 * t) - c2) - r2,
    ]

    nrm1 = esym.norm((n1 * t) - c1)
    nrm2 = esym.norm((n1 * t) - c2)
    soln = sympy.solve(facts_2, t)
    print soln

    import IPython; IPython.embed()