#
#       transform.py
#
#       Copyright 2010-2011 Enrico Avventi <avventi@kth.se>
#
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License version 2 as
#       published by the Free Software Foundation.
#
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.

from . import _wrapper
from .exceptions import raise_if_slycot_error, SlycotParameterError

import numpy as _np

def tb01id(n,m,p,maxred,a,b,c,job='A'):
    """ s_norm,A,B,C,scale = tb01id(n,m,p,maxred,A,B,C,[job])

    To reduce the 1-norm of a system matrix

          S =  ( A  B )
               ( C  0 )

    corresponding to the triple (A,B,C), by balancing. This involves
    a diagonal similarity transformation inv(D)*A*D applied
    iteratively to A to make the rows and columns of
                        -1
               diag(D,I)  * S * diag(D,I)

    as close in norm as possible.

    The balancing can be performed optionally on the following
    particular system matrices

           S = A,    S = ( A  B )    or    S = ( A )
                                               ( C )

    Required arguments:
        n : input int
            The order of the matrix A, the number of rows of matrix B and
            the number of columns of matrix C. It represents the dimension of
            the state vector.  n > 0.
        m : input int
            The number of columns of matrix B. It represents the dimension of
            the input vector.  m > 0.
        p : input int
            The number of rows of matrix C. It represents the dimension of
            the output vector.  p > 0.
        maxred : input float
            The maximum allowed reduction in the 1-norm of S  (in an iteration)
            if zero rows or columns are encountered.
            If maxred > 0.0, maxred must be larger than one (to enable the norm
            reduction).
            If maxred <= 0.0, then the value 10.0 for maxred is used.
        A : input rank-2 array('d') with bounds (n,n)
            The leading n-by-n part of this array must contain the system state
            matrix A.
        B : input rank-2 array('d') with bounds (n,m)
            The leading n-by-m part of this array must contain the system input
            matrix B.
        C : input rank-2 array('d') with bounds (p,n)
            The leading p-by-n part of this array must contain the system output
            matrix C.
    Optional arguments:
        job := 'A' input string(len=1)
            Indicates which matrices are involved in balancing, as follows:
            = 'A':  All matrices are involved in balancing;
            = 'B':  B and A matrices are involved in balancing;
            = 'C':  C and A matrices are involved in balancing;
            = 'N':  B and C matrices are not involved in balancing.
    Return objects:
        s_norm : float
            The 1-norm of the given matrix S is non-zero, the ratio between
            the 1-norm of the given matrix and the 1-norm of the balanced matrix.
        A : rank-2 array('d') with bounds (n,n)
            The leading n-by-n part of this array contains the balanced matrix
            inv(D)*A*D.
        B : rank-2 array('d') with bounds (n,m)
            The leading n-by-m part of this array contains the balanced matrix
            inv(D)*B.
        C : rank-2 array('d') with bounds (p,n)
            The leading p-by-n part of this array contains the balanced matrix C*D.
        scale : rank-1 array('d') with bounds (n)
            The scaling factors applied to S.  If D(j) is the scaling factor
            applied to row and column j, then scale(j) = D(j), for j = 1,...,n.
    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['job', 'N', 'M', 'P', 'maxred', 'A', 'LDA'+hidden, 'B',
        'LDB'+hidden, 'C', 'LDC'+hidden, 'scale', 'INFO'+hidden]
    out = _wrapper.tb01id(n,m,p,maxred,a,b,c,job=job)
    raise_if_slycot_error(out[-1], arg_list)
    return out[:-1]

def tb03ad(n,m,p,A,B,C,D,leri,equil='N',tol=0.0,ldwork=None):
    """ A_min,b_min,C_min,nr,index,pcoeff,qcoeff,vcoeff = tb03ad_l(n,m,p,A,B,C,D,leri,[equil,tol,ldwork])

    To find a relatively prime left or right polynomial matrix representation
    with the same transfer matrix as that of a given state-space representation,
    i.e. if leri = 'L'

        inv(P(s))*Q(s) = C*inv(s*I-A)*B + D

    or, if leri = 'R'

        Q(s)*inv(P(s)) = C*inv(s*I-A)*B + D.

    Additionally a minimal realization (A_min,B_min,C_min) of the original
    system (A,B,C) is returned.

    Required arguments:
        n : input int
            The order of the state-space representation, i.e. the order of
            the original state dynamics matrix A.  n > 0.
        m : input int
            The number of system inputs.  m > 0.
        p : input int
            The number of system outputs.  p > 0.
        A : input rank-2 array('d') with bounds (n,n)
            The leading n-by-n part of this array must contain the original
            state dynamics matrix A.
        B : input rank-2 array('d') with bounds (n,max(m,p))
            The leading n-by-m part of this array must contain the original
            input/state matrix B; the remainder of the leading n-by-max(m,p)
            part is used as internal workspace.
        C : input rank-2 array('d') with bounds (max(m,p),n)
            The leading p-by-n part of this array must contain the original
            state/output matrix C; the remainder of the leading max(m,p)-by-n
            part is used as internal workspace.
        D : input rank-2 array('d') with bounds (max(m,p),max(m,p))
            The leading p-by-m part of this array must contain the original
            direct transmission matrix D; the remainder of the leading
            max(m,p)-by-max(m,p) part is used as internal workspace.
        leri : input string(len=1)
            Indicates whether the left polynomial matrix representation or
            the right polynomial matrix representation is required.
    Optional arguments:
        equil := 'N' input string(len=1)
            Specifies whether the user wishes to balance the triplet (A,B,C),
            before computing a minimal state-space representation, as follows:
            = 'S':  Perform balancing (scaling);
            = 'N':  Do not perform balancing.
        tol := 0.0 input float
            The tolerance to be used in rank determination when transforming
            (A, B). If tol <= 0 a default value is used.
        ldwork := max(2*n+3*max(m,p), p*(p+2)) input int
            The length of the cache array.
            ldwork >= max( n + max(n, 3*m, 3*p), pm*(pm + 2))
            where pm = p, if leri = 'L';
                  pm = m, if leri = 'R'.
            For optimum performance it should be larger.
    Return objects:
        A_min : rank-2 array('d') with bounds (n,n)
            The leading nr-by-nr part of this array contains the upper block
            Hessenberg state dynamics matrix A_min of a minimal realization for
            the original system.
        B_min : rank-2 array('d') with bounds (n,max(m,p))
            The leading nr-by-m part of this array contains the transformed
            input/state matrix B_min.
        C_min : rank-2 array('d') with bounds (max(m,p),n)
            The leading p-by-nr part of this array contains the transformed
            state/output matrix C_min.
        nr : int
            The order of the minimal state-space representation
            (A_min,B_min,C_min).
        index : rank-1 array('i') with bounds either (p) or (m)
            If leri = 'L', index(i), i = 1,2,...,p, contains the maximum degree
            of the polynomials in the i-th row of the denominator matrix P(s)
            of the left polynomial matrix representation. These elements are
            ordered so that index(1) >= index(2) >= ... >= index(p).
            If leri = 'R', index(i), i = 1,2,...,m, contains the maximum degree
            of the polynomials in the i-th column of the denominator matrix P(s)
            of the right polynomial matrix representation. These elements are
            ordered so that index(1) >= index(2) >= ... >= index(m).
        pcoeff : rank-3 array('d') with bounds either (p,p,n+1) or (m,m,n+1)
            If leri = 'L' then porm = p, otherwise porm = m.
            The leading porm-by-porm-by-kpcoef part of this array contains
            the coefficients of the denominator matrix P(s), where
            kpcoef = max(index) + 1.
            pcoeff(i,j,k) is the coefficient in s**(index(iorj)-k+1) of
            polynomial (i,j) of P(s), where k = 1,2,...,kpcoef; if leri = 'L'
            then iorj = I, otherwise iorj = J. Thus for leri = 'L',
            P(s) = diag(s**index)*(pcoeff(.,.,1)+pcoeff(.,.,2)/s+...).
        qcoeff : rank-3 array('d') with bounds (p,m,n + 1) or (max(m,p),max(m,p))
            If leri = 'L' then porp = m, otherwise porp = p.
            If leri = 'L', the leading porm-by-porp-by-kpcoef part of this array
            contains the coefficients of the numerator matrix Q(s).
            If leri = 'R', the leading porp-by-porm-by-kpcoef part of this array
            contains the coefficients of the numerator matrix Q(s).
            qcoeff(i,j,k) is defined as for pcoeff(i,j,k).
        vcoeff : rank-3 array('d') with bounds (p,n,n+1) or (m,n,n+1)
            The leading porm-by-nr-by-kpcoef part of this array contains
            the coefficients of the intermediate matrix V(s).
            vcoeff(i,j,k) is defined as for pcoeff(i,j,k).
    Raises
    ------
    SlycotArithmeticError
        :info == 1:
            A singular matrix was encountered during the
            computation of V(s);
        :info == 2:
            A singular matrix was encountered during the
            computation of P(s).

    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['leri', 'equil', 'n', 'm', 'P', 'A', 'LDA'+hidden, 'B',
        'LDB'+hidden, 'C', 'LDC'+hidden, 'D', 'LDD'+hidden, 'nr', 'index',
        'pcoeff', 'LDPCO1'+hidden, 'LDPCO2'+hidden, 'qcoeff', 'LDQCO1'+hidden,
        'LDQCO2'+hidden, 'vcoeff', 'LDVCO1'+hidden, 'LDVCO2'+hidden, 'tol',
        'IWORK'+hidden, 'DWORK'+hidden, 'ldwork', 'INFO'+hidden]
    wfun = {"L": _wrapper.tb03ad_l,
            "R": _wrapper.tb03ad_r}
    mp_ = {"L": p, "R": m}
    mp = mp_[leri]
    if leri not in wfun.keys():
        raise SlycotParameterError('leri must be either L or R', -1)
    if ldwork is None:
        ldwork = max(2*n + 3*max(m, p), mp*(mp+2))
    out = wfun[leri](n, m, p, A, B, C, D, equil=equil, tol=tol, ldwork=ldwork)
    raise_if_slycot_error(out[-1], arg_list)
    return out[:-1]


def tb04ad(n,m,p,A,B,C,D,tol1=0.0,tol2=0.0,ldwork=None):
    """ Ar,Br,Cr,nr,denom_degs,denom_coeffs,num_coeffs = tb04ad(n,m,p,A,B,C,D,[tol1,tol2,ldwork])

    Convert a state-space system to a tranfer function or matrix of transfer functions.
    The transfer function is given as rows over common denominators.

    Required arguments
    ------------------

        n : integer
            state dimension
        m : integer
            input dimension
        p : integer
            output dimension
        A :  rank-2 array, shape(n,n)
            state dynamics matrix.
        B : rank-2 array, shape (n,m)
            input matrix
        C : rank-2 array, shape (p,n)
            output matri
        D : rank-2 array, shape (p,m)
            direct transmission matrix

     Optional arguments
     ------------------

        tol1 = 0.0: double
            tolerance in determining the transfer function coefficients,
            when set to 0, a default value is used
        tol2 = 0.0: double
            tolerance in separating out a controllable/observable subsystem
            of (A,B,C), when set to 0, a default value is used
        ldwork : int
            The length of the cache array. The default values is
            max(1,n*(n+1)+max(n*m+2*n+max(n,p),max(3*m,p)))

     Returns
     -------

        nr : int
            state dimension of the controllable subsystem
        Ar :  rank-2 array, shape(nr,nr)
            state dynamics matrix of the controllable subsystem
        Br : rank-2 array, shape (nr,m)
            input matrix of the controllable subsystem
        Cr : rank-2 array, shape (p,nr)
            output matri of the controllable subsystem
        index : rank-1 array, shape (p)
            array of orders of the denominator polynomials
        dcoeff : rank-2 array, shape (p,max(index)+1)
            array of denominator coefficients
        ucoeff : rank-3 array, shape (p,m,max(index)+1)
            array of numerator coefficients

    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['rowcol','n','m','p','A','lda'+hidden,'B','ldb'+hidden,'C','ldc'+hidden,'D', 'ldd'+hidden,
        'nr','index','dcoeff','lddcoe'+hidden, 'ucoeff','lduco1'+hidden,'lduco2'+hidden,'tol1','tol2','iwork'+hidden,'dwork'+hidden,'ldwork','info'+hidden]

    mp, pm = m, p
    porm, porp = p, m
    if ldwork is None:
        ldwork = max(1,n*(n+1)+max(n*mp+2*n+max(n,mp),3*mp,pm))
    if B.shape != (n, m):
        raise SlycotParameterError("The shape of B is ({}, {}), "
                                   "but expected ({}, {})"
                                   "".format(*(B.shape + (n, m))),
                                   -7)
    if C.shape != (p, n):
        raise SlycotParameterError("The shape of C is ({}, {}), "
                                   "but expected ({}, {})"
                                   "".format(*(C.shape + (p, n))),
                                   -9)
    if D.shape != (max(1, p), m):
        raise SlycotParameterError("The shape of D is ({}, {}), "
                                   "but expected ({}, {})"
                                   "".format(*(D.shape + (max(1, p), m))),
                                   -11)
    out = _wrapper.tb04ad_r(n,m,p,A,B,C,D,tol1,tol2,ldwork)

    raise_if_slycot_error(out[-1], arg_list)

    A,B,C,Nr,index,dcoeff,ucoeff = out[:-1]
    kdcoef = max(index)+1
    return A[:Nr,:Nr],B[:Nr,:m],C[:p,:Nr],Nr,index,dcoeff[:porm,:kdcoef],ucoeff[:porm,:porp,:kdcoef]


def tb05ad(n, m, p, jomega, A, B, C, job='NG'):
    """tb05ad(n, m, p, jomega, A, B, C, job='NG')

    To find the complex frequency response matrix (transfer matrix)
    G(freq) of the state-space representation (A,B,C) given by

    ::

                                  -1
       G(freq) = C * ((freq*I - A)  ) * B

    where A, B and C are real N-by-N, N-by-M and P-by-N matrices
    respectively and freq is a complex scalar.

    Parameters
    ----------
    n : int
        The number of states, i.e. the order of the state
        transition matrix A.
    m : int
        The number of inputs, i.e. the number of columns in the
        matrix B.
    p :  int
        The number of outputs, i.e. the number of rows in the
        matrix C.
    jomega : complex float
        The frequency at which the frequency response matrix
        (transfer matrix) is to be evaluated. For continuous time
        systems, this is j*omega, where omega is the frequency to
        be evaluated. For discrete time systems,
        freq = exp(j*omega*Ts)
    A : (n,n) ndarray
        On entry, this array must contain the state transition
        matrix A.
    B : (n,m) ndarray
        On entry, this array must contain the input/state matrix B.
    C : (p,n) ndarray
        On entry, of this array must contain the state/output matrix C.
    job : {'AG', 'NG', 'NH'}
        If job = 'AG' (i.e., 'all', 'general matrix'), the A matrix is
        first balanced. The balancing transformation
        is then appropriately applied to matrices B and C. The A matrix
        is (again) transformed to an upper Hessenberg representation and
        the B and C matrices are also transformed. In addition,
        the condition number of the problem is calculated as well as the
        eigenvalues of A.

        If job='NG' (i.e., 'none', 'general matrix'), no balancing is done.
        Neither the condition number nor the eigenvalues are calculated.
        The routine still transforms A into upper Hessenberg form. The
        matrices B and C are also appropriately transformed.

        If job = 'NH' (i.e., 'none', 'hessenberg matrix'), the function
        assumes the matrices have already been transformed into Hessenberg
        form, i.e., by a previous function call tb05ad. If this not the
        case, the routine will return a wrong result without warning.

    Returns
    -------
    if job = 'AG':
    --------------
      At:  The A matrix which has been both balanced and
           transformed to upper Hessenberg form. The balancing
           transforms A according to
                      A1 =   P^-1 * A * P.
           The transformation to upper Hessenberg form then yields
                      At = Q^T * (P^-1 * A * P ) * Q.
           Note that the lower triangle of At is in general not zero.
           Rather, it contains information on the orthogonal matrix Q
           used to transform A1 to Hessenberg form. See docs for lappack
           DGEHRD():
           http://www.netlib.org/lapack/explore-3.1.1-html/dgehrd.f.html
           However, it does not apparently contain information on P, the
           matrix used in the balancing procedure.

      Bt:  The matrix B transformed according to
                      Bt = Q^T * P^-1 * B.

      Ct:  The matrix C transformed according to
                      Ct = C * P * Q

      rcond: RCOND contains an estimate of the reciprocal of the
             condition number of matrix H with respect to inversion, where
                     H = (j*freq * I - A)

      g_jw: complex p-by-m array, which contains the frequency response
            matrix G(freq).

      ev:   Eigenvalues of the matrix A.

      hinvb : complex n-by-m array, which  contains the product
               -1
              H  B.

    if job = 'NG':
    --------------
      At:    The matrix A transformed to upper Hessenberg form according
             to
                      At = Q^T  * A  * Q.
             The lower triangle is not zero. It containts info on the
             orthoganal transformation. See docs for linpack DGEHRD()
             http://www.netlib.org/lapack/explore-3.1.1-html/dgehrd.f.html

      Bt:    The matrix B transformed according to
                      Bt = Q^T * B.

      Ct:    The matrix C transformed according to
                      Ct = C * Q
      g_jw:  complex array with dim p-by-m which contains the frequency
             response matrix G(freq).

      hinvb : complex array with dimension p-by-m.
              This array contains the
                      -1
             product H  B.

    if job = 'NH'
    --------------
      g_jw:  complex p-by-m array which contains the frequency
             response matrix G(freq).

      hinvb : complex p-by-m array which contains the
                      -1
             product H  B.


    Raises
    ------
    SlycotArithmeticError
        :info = 1:
            More than {n30} (30*`n`) iterations were required to isolate the
            eigenvalues of A. The computations are continued.
        :info = 2:
            Either `freq`={jomega} is too near to an eigenvalue of A,
            or `rcond` is less than the machine precision EPS.

    Example
    -------
    >>> A = np.array([[0.0, 1.0],
                [-100.0,   -20.1]])
    >>> B = np.array([[0.],[100]])
    >>> C = np.array([[1., 0.]])
    >>> n = np.shape(A)[0]
    >>> m = np.shape(B)[1]
    >>> p = np.shape(C)[0]
    >>> jw_s = [1j*10, 1j*15]
    >>> at, bt, ct, g_1, hinvb, info = slycot.tb05ad(n, m, p, jw_s[0],
                                                    A, B, C, job='NG')
    >>> g_2, hinv2,info = slycot.tb05ad(n, m, p, jw_s[1], at, bt, ct, job='NH')

    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['baleig'+hidden, 'inita'+hidden, 'n', 'm', 'p', 'freq', 'a',
                'lda'+hidden, 'b', 'ldb'+hidden, 'c', 'ldc'+hidden, 'rcond',
                'g', 'ldg'+hidden, 'evre', 'evim', 'hinvb', 'ldhinv'+hidden,
                'iwork'+hidden, 'dwork'+hidden, 'ldwork'+hidden,
                'zwork'+hidden, 'lzwork'+hidden, 'info'+hidden]
    # Fortran function prototype:
    # TB05AD(baleig,inita,n,m,p,freq,a,lda,b,ldb,c,ldc,rcond,g,ldg,evre,evim,hinvb,ldhinv,
    # iwork,dwork,ldwork,zwork,lzwork,info)

    # Sanity check on matrix dimensions
    if A.shape != (n, n):
        raise SlycotParameterError("The shape of A is ({0:}, {1:}), "
                                   "but expected ({2:}, {2:})"
                                   "".format(*(A.shape + (n,))),
                                   -7)
    if B.shape != (n, m):
        raise SlycotParameterError("The shape of B is ({0:}, {1:}), "
                                   "but expected ({2:}, {3:})"
                                   "".format(*(B.shape + (n, m))),
                                   -9)
    if C.shape != (p, n):
        raise SlycotParameterError("The shape of C is ({0:}, {1:}), "
                                   "but expected ({2:}, {3:})"
                                   "".format(*(C.shape + (p, n))),
                                   -11)

    # ----------------------------------------------------
    # Checks done, do computation.
    n30 = 30*n  # for INFO = 1 error docstring
    if job == 'AG':
        out = _wrapper.tb05ad_ag(n, m, p, jomega, A, B, C)
        At, Bt, Ct, rcond, g_jw, evre, evim, hinvb, info = out
        raise_if_slycot_error(info, arg_list, tb05ad.__doc__, locals())
        ev = _np.zeros(n, 'complex64')
        ev.real = evre
        ev.imag = evim
        return At, Bt, Ct, g_jw, rcond, ev, hinvb, info
    elif job == 'NG':
        # use tb05ad_ng, for 'NONE' , and 'General', because balancing
        # (option 'A' for 'ALL') seems to have  a bug.
        out = _wrapper.tb05ad_ng(n, m, p, jomega, A, B, C)
        At, Bt, Ct, g_jw, hinvb, info = out
        raise_if_slycot_error(info, arg_list, tb05ad.__doc__, locals())
        return At, Bt, Ct, g_jw, hinvb, info
    elif job == 'NH':
        out = _wrapper.tb05ad_nh(n, m, p, jomega, A, B, C)
        g_i, hinvb, info = out
        raise_if_slycot_error(info, arg_list, tb05ad.__doc__, locals())
        return g_i, hinvb, info
    else:
        raise SlycotParameterError("Unrecognized job. Expected job = 'AG' or "
                                   "job='NG' or job = 'NH' but received job={}"
                                   "".format(job),
                                   -1)  # job is baleig and inita together


def td04ad(rowcol,m,p,index,dcoeff,ucoeff,tol=0.0,ldwork=None):
    """ nr,A,B,C,D = td04ad(rowcol,m,p,index,dcoeff,ucoeff,[tol,ldwork])

    Convert a transfer function or matrix of transfer functions to
    a minimum state space realization.

    Parameters
    ----------
    rowcol : {R', 'C'}
        indicates whether the transfer matrix T(s) is given
        as rows ('R') or colums ('C') over common denominators.
    m : int
        input dimension
    p : int
        output dimension
    index : (p,) or (m,) array_like
        array of orders of the denominator polynomials. Different
        shapes corresponding to rowcol=='R' and rowcol=='C'
        respectively.
    dcoeff : (p,max(index)+1) or (m,max(index)+1) ndarray
        array of denominator coefficients. Different shapes
        corresponding to rowcol=='R' and rowcol=='C' respectively.
    ucoeff : (p,m,max(index)+1) or (max(p,m),max(p,m),max(index)+1) ndarray
        array of numerator coefficients. Different shapes
        corresponding to rowcol=='R' and rowcol=='C' respectively.
    tol : float, optional
        tolerance in determining the state space system,
        when set to 0, a default value is used.
    ldwork : int, optional
        The length of the cache array. The default values is
        max(1,sum(index)+max(sum(index),max(3*m,3*p)))

    Returns
    -------
    nr : int
        minimal state dimension
    A : (nr,nr) ndarray
        state dynamics matrix.
    B : (nr,m) ndarray
        input matrix
    C : (p,nr) ndarray
        output matrix
    D : (p,m) ndarray
        direct transmission matrix

    Raises
    ------
    SlycotArithmeticError
        :info > 0:
            i={info} is the first index of `dcoeff` for which
            ``abs( dcoeff(i,1) )`` is so small that the calculations
            would overflow (see SLICOT Library routine TD03AY);
            that is, the leading coefficient of a polynomial is
            nearly zero;
    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['rowcol','m','p','index','dcoeff','lddcoe'+hidden, 'ucoeff', 'lduco1'+hidden,'lduco2'+hidden,
        'nr','A','lda'+hidden,'B','ldb'+hidden,'C','ldc'+hidden,'D', 'ldd'+hidden,
        'tol','iwork'+hidden,'dwork'+hidden,'ldwork','info'+hidden]
    if ldwork is None:
        n = sum(index)
        ldwork = max(1,n+max(n,max(3*m,3*p)))

    kdcoef = max(index)+1
    if rowcol == 'R':
        if ucoeff.ndim != 3:
            raise SlycotParameterError("The numerator is not a 3D array!", -7)
        expectedshape = (max(1, p), max(1, m), kdcoef)
        if ucoeff.shape != expectedshape:
            raise SlycotParameterError("The numerator shape is ({}, {}, {}), "
                                       "but expected ({}, {}, {})".format(
                                           *(ucoeff.shape + expectedshape)),
                                       -7)
        expectedshape = (max(1, p), kdcoef)
        if dcoeff.shape != expectedshape:
            raise SlycotParameterError("The denominator shape is ({}, {}), "
                                       "but expected ({}, {})".format(
                                           *(dcoeff.shape + expectedshape)),
                                       -5)
        out = _wrapper.td04ad_r(m,p,index,dcoeff,ucoeff,n,tol,ldwork)
    elif rowcol == 'C':
        if ucoeff.ndim != 3:
            raise SlycotParameterError("The numerator is not a 3D array!", -7)
        expectedshape = (max(1, m, p), max(1, m, p), kdcoef)
        if ucoeff.shape != expectedshape:
            raise SlycotParameterError("The numerator shape is ({}, {}, {}), "
                                       "but expected ({}, {}, {})".format(
                                           *(ucoeff.shape + expectedshape)),
                                       -7)
        expectedshape = (max(1, m), kdcoef)
        if dcoeff.shape != expectedshape:
            raise SlycotParameterError("The denominator shape is ({}, {}), "
                                       "but expected ({}, {})".format(
                                           *(dcoeff.shape + expectedshape)),
                                       -5)
        out = _wrapper.td04ad_c(m,p,index,dcoeff,ucoeff,n,tol,ldwork)
    else:
        raise SlycotParameterError("Parameter rowcol had an illegal value", -1)

    raise_if_slycot_error(out[-1], arg_list, td04ad.__doc__)
    Nr, A, B, C, D = out[:-1]
    return Nr, A[:Nr,:Nr], B[:Nr,:m], C[:p,:Nr], D[:p,:m]

def tc04ad(m,p,index,pcoeff,qcoeff,leri,ldwork=None):
    """ n,rcond,a,b,c,d = tc04ad_l(m,p,index,pcoeff,qcoeff,leri,[ldwork])

    To find a state-space representation (A,B,C,D) with the same
    transfer matrix as that of a given left or right polynomial
    matrix representation, i.e.

        C*inv(sI-A)*B + D = inv(P(s))*Q(s)

    or

        C*inv(sI-A)*B + D = Q(s)*inv(P(s))

    respectively.

    Required arguments:
        m : input int
            The number of system inputs.  m > 0.
        p := len(index) input int
            The number of system outputs.  p > 0.
        index : input rank-1 array('i') with bounds (p) or (m)
            If leri = 'L', index(i), i = 1,2,...,p, must contain the maximum
            degree of the polynomials in the I-th row of the denominator matrix
            P(s) of the given left polynomial matrix representation.
            If leri = 'R', index(i), i = 1,2,...,m, must contain the maximum
            degree of the polynomials in the I-th column of the denominator
            matrix P(s) of the given right polynomial matrix representation.
        pcoeff : input rank-3 array('d') with bounds (p,p,*) or (m,m,*)
            If leri = 'L' then porm = p, otherwise porm = m. The leading
            porm-by-porm-by-kpcoef part of this array must contain
            the coefficients of the denominator matrix P(s). pcoeff(i,j,k) is
            the coefficient in s**(index(iorj)-K+1) of polynomial (I,J) of P(s),
            where k = 1,2,...,kpcoef and kpcoef = max(index) + 1; if leri = 'L'
            then iorj = i, otherwise iorj = j. Thus for leri = 'L',
            P(s) = diag(s**index)*(pcoeff(.,.,1)+pcoeff(.,.,2)/s+...).
            If leri = 'R', pcoeff is modified by the routine but restored on exit.
        qcoeff : input rank-3 array('d') with bounds (p,m,*) or (max(m,p),max(m,p),*)
            If leri = 'L' then porp = m, otherwise porp = p. The leading
            porm-by-porp-by-kpcoef part of this array must contain
            the coefficients of the numerator matrix Q(s).
            qcoeff(i,j,k) is defined as for pcoeff(i,j,k).
            If leri = 'R', qcoeff is modified by the routine but restored on exit.
        leri : input string(len=1)
            Indicates whether a left polynomial matrix representation or a right
            polynomial matrix representation is input as follows:
            = 'L':  A left matrix fraction is input;
            = 'R':  A right matrix fraction is input.
    Optional arguments:
        ldwork := max(m,p)*(max(m,p)+4) input int
            The length of the cache array. ldwork >= max(m,p)*(max(m,p)+4)
            For optimum performance it should be larger.
    Return objects:
        n : int
            The order of the resulting state-space representation.
            That is, n = sum(index).
        rcond : float
            The estimated reciprocal of the condition number of the leading row
            (if leri = 'L') or the leading column (if leri = 'R') coefficient
            matrix of P(s).
            If rcond is nearly zero, P(s) is nearly row or column non-proper.
        A : rank-2 array('d') with bounds (n,n)
            The leading n-by-n part of this array contains the state dynamics matrix A.
        B : rank-2 array('d') with bounds (n,max(m,p))
            The leading n-by-n part of this array contains the input/state matrix B;
            the remainder of the leading n-by-max(m,p) part is used as internal
            workspace.
        C : rank-2 array('d') with bounds (max(m,p),n)
            The leading p-by-n part of this array contains the state/output matrix C;
            the remainder of the leading max(m,p)-by-n part is used as internal
            workspace.
        D : rank-2 array('d') with bounds (max(m,p),max(m,p))
            The leading p-by-m part of this array contains the direct transmission
            matrix D; the remainder of the leading max(m,p)-by-max(m,p) part is
            used as internal workspace.
    Raises
    ------
    SlycotArithmeticError
        :info == 1 and leri = 'L':
            P(s) is not row proper
        :info == 1 and leri = 'R':
            P(s) is not column proper
    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['leri', 'm', 'P', 'index',
                'pcoeff', 'LDPCO1' + hidden, 'LDPCO2' + hidden,
                'qcoeff', 'LDQCO1' + hidden, 'LDQCO2' + hidden,
                'N', 'rcond',
                'A', 'LDA' + hidden, 'B', 'LDB' + hidden,
                'C', 'LDC' + hidden, 'D', 'LDD' + hidden,
                'IWORK' + hidden, 'DWORK' + hidden, 'ldwork',
                'INFO' + hidden]
    if ldwork is None:
        ldwork = max(m, p)*(max(m, p)+4)
    n = sum(index)
    wfun = {"L": _wrapper.tc04ad_l, "R": _wrapper.tc04ad_r}
    if leri not in wfun.keys():
        raise SlycotParameterError('leri must be either L or R', -1)
    out = wfun[leri](m, p, index, pcoeff, qcoeff, n)
    raise_if_slycot_error(out[-1], arg_list, tc04ad.__doc__, locals())
    return out[:-1]


def tc01od(m,p,indlin,pcoeff,qcoeff,leri):
    """ pcoeff,qcoeff = tc01od_l(m,p,indlim,pcoeff,qcoeff,leri)

    To find the dual right (left) polynomial matrix representation of a given
    left (right) polynomial matrix representation, where the right and left
    polynomial matrix representations are of the form Q(s)*inv(P(s)) and
    inv(P(s))*Q(s) respectively.

    Required arguments:
        m : input int
            The number of system inputs.  m > 0.
        p : input int
            The number of system outputs.  p > 0.
        indlim : input int
            The highest value of k for which pcoeff(.,.,k) and qcoeff(.,.,k)
            are to be transposed.
            k = kpcoef + 1, where kpcoef is the maximum degree of the polynomials
            in P(s).  indlim > 0.
        pcoeff : input rank-3 array('d') with bounds (p,p,indlim) or (m,m,indlim)
            If leri = 'L' then porm = p, otherwise porm = m.
            On entry, the leading porm-by-porm-by-indlim part of this array
            must contain the coefficients of the denominator matrix P(s).
            pcoeff(i,j,k) is the coefficient in s**(indlim-k) of polynomial
            (i,j) of P(s), where k = 1,2,...,indlim.
        qcoeff : input rank-3 array('d') with bounds (max(m,p),max(m,p),indlim)
            On entry, the leading p-by-m-by-indlim part of this array must
            contain the coefficients of the numerator matrix Q(s).
            qcoeff(i,j,k) is the coefficient in s**(indlim-k) of polynomial
            (i,j) of Q(s), where k = 1,2,...,indlim.
        leri : input string(len=1)
    Return objects:
        pcoeff : rank-3 array('d') with bounds (p,p,indlim)
            On exit, the leading porm-by-porm-by-indlim part of this array
            contains the coefficients of the denominator matrix P'(s) of
            the dual system.
        qcoeff : rank-3 array('d') with bounds (max(m,p),max(m,p),indlim)
            On exit, the leading m-by-p-by-indlim part of the array contains
            the coefficients of the numerator matrix Q'(s) of the dual system.
        info : int
            = 0:  successful exit;
            < 0:  if info = -i, the i-th argument had an illegal value.
    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['leri', 'M', 'P', 'indlim', 'pcoeff', 'LDPCO1'+hidden,
        'LDPCO2'+hidden, 'qcoeff', 'LDQCO1'+hidden, 'LDQCO2'+hidden,
        'INFO'+hidden]
    if leri == 'L':
        out = _wrapper.tc01od_l(m,p,indlin,pcoeff,qcoeff)
        raise_if_slycot_error(out[-1], arg_list)
        return out[:-1]
    if leri == 'R':
        out = _wrapper.tc01od_r(m,p,indlin,pcoeff,qcoeff)
        raise_if_slycot_error(out[-1], arg_list)
        return out[:-1]
    raise SlycotParameterError('leri must be either L or R', -1)

def tf01md(n,m,p,N,A,B,C,D,u,x0):
    """ xf,y = tf01md(n,m,p,N,A,B,C,D,u,x0)

    To compute the output sequence of a linear time-invariant
    open-loop system given by its discrete-time state-space model

    Required arguments:
        n : input int
            Order of the State-space representation.
        m : input int
            Number of inputs.
        p : input int
            Number of outputs.
        N : input int
            Number of output samples to be computed.
        A : input rank-2 array('d') with bounds (n,n)
            State dynamics matrix.
        B : input rank-2 array('d') with bounds (n,m)
            Input/state matrix.
        C : input rank-2 array('d') with bounds (p,n)
            State/output matrix.
        D : input rank-2 array('d') with bounds (p,m)
            Direct transmission matrix.
        u : input rank-2 array('d') with bounds (m,N)
            Input signal.
        x0 : input rank-1 array('d') with bounds (n)
            Initial state, at time 0.
    Return objects:
        xf : rank-1 array('d') with bounds (n)
            Final state, at time N+1.
        y : rank-2 array('d') with bounds (p,N)
            Output signal.
    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['n','m','p','ny','A','lda'+hidden,'B','ldb'+hidden,
        'C','ldc'+hidden,'D','ldd'+hidden,'u','ldu'+hidden,'x0',
        'y'+hidden,'ldy'+hidden,'dwork'+hidden,'info'+hidden]

    out = _wrapper.tf01md(n,m,p,N,A,B,C,D,u,x0)
    raise_if_slycot_error(out[-1], arg_list)
    return out[:-1]

def tf01rd(n,m,p,N,A,B,C,ldwork=None):
    """ H = tf01rd(n,m,p,N,A,B,C,[ldwork])

    To compute N Markov parameters M_1, M_2,..., M_N from the
    parameters (A,B,C) of a linear time-invariant system, where each
    M_k is an p-by-m matrix and k = 1,2,...,N.

    All matrices are treated as dense, and hence TF01RD is not
    intended for large sparse problems.


    Required arguments:
        n : input int
            Order of the State-space representation.
        m : input int
            Number of inputs.
        p : input int
            Number of outputs.
        N : input int
            Number of Markov parameters to be computed.
        A : input rank-2 array('d') with bounds (n,n)
            State dynamics matrix.
        B : input rank-2 array('d') with bounds (n,m)
            Input/state matrix.
        C : input rank-2 array('d') with bounds (p,n)
            State/output matrix.
    Optional arguments:
        ldwork := 2*na*nc input int
    Return objects:
        H : rank-2 array('d') with bounds (p,N*m)
            H[:,(k-1)*m : k*m] contains the k-th Markov parameter,
            for k = 1,2...N.
    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['n','m','p','N','A','lda'+hidden,'B','ldb'+hidden,'C',
        'ldc'+hidden,'H','ldh'+hidden,'dwork'+hidden,'ldwork','info'+hidden]

    if ldwork is None:
        out = _wrapper.tf01rd(n,m,p,N,A,B,C)
    else:
        out = _wrapper.tf01rd(n,m,p,N,A,B,C,ldwork=ldwork)
    raise_if_slycot_error(out[-1], arg_list)
    return out[0]

def tb01pd(n, m, p, A, B, C, job='M', equil='S', tol=1e-8, ldwork=None):
    """Ar, Br, Cr, nr = tb01pd(n,m,p,A,B,C,[job,equil,tol,ldwork])

    To find a reduced (controllable, observable, or minimal) state-
    space representation (Ar,Br,Cr) for any original state-space
    representation (A,B,C). The matrix Ar is in upper block
    Hessenberg form.

    Required arguments:
        n : input int
            Order of the State-space representation.
        m : input int
            Number of inputs.
        p : input int
            Number of outputs.
        A : input rank-2 array('d') with bounds (n,n)
            State dynamics matrix.
        B : input rank-2 array('d') with bounds (n,max(m,p))
            The leading n-by-m part of this array must contain the original
            input/state matrix B; the remainder of the leading n-by-max(m,p)
            part is used as internal workspace.
        C : input rank-2 array('d') with bounds (p,n)
            The leading p-by-n part of this array must contain the original
            state/output matrix C; the remainder of the leading max(1,m,p)-by-n
            part is used as internal workspace.
    Optional arguments:
        job : input char*1
            Indicates whether the user wishes to remove the
            uncontrollable and/or unobservable parts as follows:
            = 'M':  Remove both the uncontrollable and unobservable
                    parts to get a minimal state-space representation;
            = 'C':  Remove the uncontrollable part only to get a
                    controllable state-space representation;
            = 'O':  Remove the unobservable part only to get an
                    observable state-space representation.
        equil : input char*1
            Specifies whether the user wishes to preliminarily balance
            the triplet (A,B,C) as follows:
            = 'S':  Perform balancing (scaling);
            = 'N':  Do not perform balancing.
    Return objects:
        Ar : output rank-2 array('d') with bounds (nr,nr)
            Contains the upper block Hessenberg state dynamics matrix
            Ar of a minimal, controllable, or observable realization
            for the original system, depending on the value of JOB,
            JOB = 'M', JOB = 'C', or JOB = 'O', respectively.
        Br : output rank-2 array('d') with bounds (nr,m)
            Contains the transformed input/state matrix Br of a
            minimal, controllable, or observable realization for the
            original system, depending on the value of JOB, JOB = 'M',
            JOB = 'C', or JOB = 'O', respectively.  If JOB = 'C', only
            the first IWORK(1) rows of B are nonzero.
        Cr : output rank-2 array('d') with bounds (p,nr)

            Contains the transformed state/output matrix Cr of a
            minimal, C controllable, or observable realization for the
            original C system, depending on the value of JOB, JOB =
            'M', C JOB = 'C', or JOB = 'O', respectively.  C If JOB =
            'M', or JOB = 'O', only the last IWORK(1) columns C (in
            the first NR columns) of C are nonzero.
        nr : output int
            The order of the reduced state-space representation
            (Ar,Br,Cr) of a minimal, controllable, or observable
            realization for the original system, depending on
            JOB = 'M', JOB = 'C', or JOB = 'O'.
    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['job', 'equil', 'n','m','p','A','lda'+hidden,'B','ldb'+hidden,
                'C','ldc'+hidden,'nr','tol','iwork'+hidden,'dwork'+hidden,
                'ldwork','info'+hidden]
    if ldwork is None:
        ldwork = max(1, n+max(n,3*m,3*p))
    elif ldwork < max(1, n+max(n,3*m,3*p)):
        raise SlycotParameterError("ldwork is too small", -15)
    out = _wrapper.tb01pd(n=n,m=m,p=p,a=A,b=B,c=C,
                          job=job,equil=equil,tol=tol,ldwork=ldwork)

    raise_if_slycot_error(out[-1], arg_list)
    return out[:-1]

def tg01ad(l,n,m,p,A,E,B,C,thresh=0.0,job='A'):
    """ A,E,B,C,lscale,rscale = tg01ad(l,n,m,p,A,E,B,C,[thresh,job])

    To balance the matrices of the system pencil

            S =  ( A  B ) - lambda ( E  0 ) :=  Q - lambda Z,
                 ( C  0 )          ( 0  0 )

    corresponding to the descriptor triple (A-lambda E,B,C),
    by balancing. This involves diagonal similarity transformations
    (Dl*A*Dr - lambda Dl*E*Dr, Dl*B, C*Dr) applied to the system
    (A-lambda E,B,C) to make the rows and columns of system pencil
    matrices

                diag(Dl,I) * S * diag(Dr,I)

    as close in norm as possible. Balancing may reduce the 1-norms
    of the matrices of the system pencil S.

    The balancing can be performed optionally on the following
    particular system pencils

            S = A-lambda E,

            S = ( A-lambda E  B ),    or

            S = ( A-lambda E ).
                (     C      )
    Required arguments:
        l : input int
            The number of rows of matrices A, B, and E.  l >= 0.
        n : input int
            The number of columns of matrices A, E, and C.  n >= 0.
        m : input int
            The number of columns of matrix B.  m >= 0.
        p : input int
            The number of rows of matrix C.  P >= 0.
        A : rank-2 array('d') with bounds (l,n)
            The leading L-by-N part of this array must
            contain the state dynamics matrix A.
        E : rank-2 array('d') with bounds (l,n)
            The leading L-by-N part of this array must
            contain the descriptor matrix E.
        B : rank-2 array('d') with bounds (l,m)
            The leading L-by-M part of this array must
            contain the input/state matrix B.
            The array B is not referenced if M = 0.
        C : rank-2 array('d') with bounds (p,n)
            The leading P-by-N part of this array must
            contain the state/output matrix C.
            The array C is not referenced if P = 0.
    Optional arguments:
        job := 'A' input string(len=1)
          Indicates which matrices are involved in balancing, as
          follows:
          = 'A':  All matrices are involved in balancing;
          = 'B':  B, A and E matrices are involved in balancing;
          = 'C':  C, A and E matrices are involved in balancing;
          = 'N':  B and C matrices are not involved in balancing.
        thresh := 0.0 input float
            Threshold value for magnitude of elements:
            elements with magnitude less than or equal to
            THRESH are ignored for balancing. THRESH >= 0.
    Return objects:
        A : rank-2 array('d') with bounds (l,n)
          The leading L-by-N part of this array contains
          the balanced matrix Dl*A*Dr.
        E : rank-2 array('d') with bounds (l,n)
          The leading L-by-N part of this array contains
          the balanced matrix Dl*E*Dr.
        B : rank-2 array('d') with bounds (l,m)
          If M > 0, the leading L-by-M part of this array
          contains the balanced matrix Dl*B.
          The array B is not referenced if M = 0.
        C : rank-2 array('d') with bounds (p,n)
          If P > 0, the leading P-by-N part of this array
          contains the balanced matrix C*Dr.
          The array C is not referenced if P = 0.
        lscale : rank-1 array('d') with bounds (l)
          The scaling factors applied to S from left.  If Dl(j) is
          the scaling factor applied to row j, then
          SCALE(j) = Dl(j), for j = 1,...,L.
        rscale : rank-1 array('d') with bounds (n)
          The scaling factors applied to S from right.  If Dr(j) is
          the scaling factor applied to column j, then
          SCALE(j) = Dr(j), for j = 1,...,N.
    """

    hidden = ' (hidden by the wrapper)'
    arg_list = ['job', 'l', 'n', 'm', 'p', 'thresh', 'A', 'lda'+hidden, 'E','lde'+hidden,'B','ldb'+hidden,'C','ldc'+hidden, 'lscale', 'rscale', 'dwork'+hidden, 'info']

    A,E,B,C,lscale,rscale,info = _wrapper.tg01ad(job,l,n,m,p,thresh,A,E,B,C)
    raise_if_slycot_error(info, arg_list)
    return A,E,B,C,lscale,rscale

def tg01fd(l,n,m,p,A,E,B,C,Q=None,Z=None,compq='N',compz='N',joba='N',tol=0.0,ldwork=None):
    """ A,E,B,C,ranke,rnka22,Q,Z = tg01fd(l,n,m,p,A,E,B,C,[Q,Z,compq,compz,joba,tol,ldwork])

    To compute for the descriptor system (A-lambda E,B,C)
    the orthogonal transformation matrices Q and Z such that the
    transformed system (Q'*A*Z-lambda Q'*E*Z, Q'*B, C*Z) is
    in a SVD-like coordinate form with

                 ( A11  A12 )             ( Er  0 )
        Q'*A*Z = (          ) ,  Q'*E*Z = (       ) ,
                 ( A21  A22 )             (  0  0 )

    where Er is an upper triangular invertible matrix.
    Optionally, the A22 matrix can be further reduced to the form

                  ( Ar  X )
            A22 = (       ) ,
                  (  0  0 )

    with Ar an upper triangular invertible matrix, and X either a full
    or a zero matrix.
    The left and/or right orthogonal transformations performed
    to reduce E and A22 can be optionally accumulated.

    Required arguments:
        l : input int
            The number of rows of matrices A, B, and E.  l >= 0.
        n : input int
            The number of columns of matrices A, E, and C.  n >= 0.
        m : input int
            The number of columns of matrix B.  m >= 0.
        p : input int
            The number of rows of matrix C.  p >= 0.
        A : rank-2 array('d') with bounds (l,n)
            The leading l-by-n part of this array must
            contain the state dynamics matrix A.
        E : rank-2 array('d') with bounds (l,n)
            The leading l-by-n part of this array must
            contain the descriptor matrix E.
        B : rank-2 array('d') with bounds (l,m)
            The leading L-by-M part of this array must
            contain the input/state matrix B.
        C : rank-2 array('d') with bounds (p,n)
            The leading P-by-N part of this array must
            contain the state/output matrix C.
    Optional arguments:
        Q : rank-2 array('d') with bounds (l,l)
            If COMPQ = 'N':  Q is not referenced.
            If COMPQ = 'I':  Q need not be set.
            If COMPQ = 'U':  The leading l-by-l part of this
                            array must contain an orthogonal matrix
                            Q1.
        Z : rank-2 array('d') with bounds (n,n)
            If COMPZ = 'N':  Z is not referenced.
            If COMPZ = 'I':  Z need not be set.
            If COMPZ = 'U':  The leading n-by-n part of this
                            array must contain an orthogonal matrix
                            Z1.
        compq := 'N' input string(len=1)
            = 'N':  do not compute Q.
            = 'I':  Q is initialized to the unit matrix, and the
                    orthogonal matrix Q is returned.
            = 'U':  Q must contain an orthogonal matrix Q1 on entry,
                    and the product Q1*Q is returned.
        compz := 'N' input string(len=1)
            = 'N':  do not compute Z.
            = 'I':  Z is initialized to the unit matrix, and the
                    orthogonal matrix Z is returned.
            = 'U':  Z must contain an orthogonal matrix Z1 on entry,
                    and the product Z1*Z is returned.
        joba := 'N' input string(len=1)
            = 'N':  do not reduce A22.
            = 'R':  reduce A22 to a SVD-like upper triangular form.
            = 'T':  reduce A22 to an upper trapezoidal form.
        tol := 0 input float
            The tolerance to be used in determining the rank of E
            and of A22. If the user sets TOL > 0, then the given
            value of TOL is used as a lower bound for the
            reciprocal condition numbers of leading submatrices
            of R or R22 in the QR decompositions E * P = Q * R of E
            or A22 * P22 = Q22 * R22 of A22.
            A submatrix whose estimated condition number is less than
            1/TOL is considered to be of full rank.  If the user sets
            TOL <= 0, then an implicitly computed, default tolerance,
            defined by  TOLDEF = L*N*EPS,  is used instead, where
            EPS is the machine precision (see LAPACK Library routine
            DLAMCH). TOL < 1.
        ldwork : input int
            The length of the cache array.
            ldwork >= MAX( 1, n+p, MIN(l,n)+MAX(3*n-1,m,l) ).
            For optimal performance, ldwork should be larger.
    Return objects:
        A : rank-2 array('d') with bounds (l,n)
            On entry, the leading L-by-N part of this array must
            contain the state dynamics matrix A.
            On exit, the leading L-by-N part of this array contains
            the transformed matrix Q'*A*Z. If JOBA = 'T', this matrix
            is in the form

                         ( A11  *   *  )
                Q'*A*Z = (  *   Ar  X  ) ,
                         (  *   0   0  )

            where A11 is a RANKE-by-RANKE matrix and Ar is a
            RNKA22-by-RNKA22 invertible upper triangular matrix.
            If JOBA = 'R' then A has the above form with X = 0.
        E : rank-2 array('d') with bounds (l,n)
            The leading L-by-N part of this array contains
            the transformed matrix Q'*E*Z.

                     ( Er  0 )
            Q'*E*Z = (       ) ,
                     (  0  0 )

            where Er is a RANKE-by-RANKE upper triangular invertible
            matrix.
        B : rank-2 array('d') with bounds (l,m)
            The leading L-by-M part of this array contains
            the transformed matrix Q'*B.
        C : rank-2 array('d') with bounds (p,n)
            The leading P-by-N part of this array contains
            the transformed matrix C*Z.
        Q : rank-2 array('d') with bounds (l,l)
            If COMPQ = 'N':  Q is not referenced.
            If COMPQ = 'I':  The leading L-by-L part of this
                            array contains the orthogonal matrix Q,
                            where Q' is the product of Householder
                            transformations which are applied to A,
                            E, and B on the left.
            If COMPQ = 'U':  The leading L-by-L part of this
                            array contains the orthogonal matrix
                            Q1*Q.
        Z : rank-2 array('d') with bounds (n,n)
            If COMPZ = 'N':  Z is not referenced.
            If COMPZ = 'I':  The leading N-by-N part of this
                            array contains the orthogonal matrix Z,
                            which is the product of Householder
                            transformations applied to A, E, and C
                            on the right.
            If COMPZ = 'U':  The leading N-by-N part of this
                            array contains the orthogonal matrix
                            Z1*Z.
        ranke : output int
            The estimated rank of matrix E, and thus also the order
            of the invertible upper triangular submatrix Er.
        rnka22 : output int
            If JOBA = 'R' or 'T', then RNKA22 is the estimated rank of
            matrix A22, and thus also the order of the invertible
            upper triangular submatrix Ar.
            If JOBA = 'N', then RNKA22 is not referenced.
    """

    hidden = ' (hidden by the wrapper)'
    arg_list = ['compq', 'compz', 'joba', 'l', 'n', 'm', 'p', 'A', 'lda'+hidden, 'E','lde'+hidden,'B','ldb'+hidden,'C','ldc'+hidden,'Q','ldq'+hidden,'Z','ldz'+hidden,'ranke','rnka22','tol','iwork'+hidden, 'dwork'+hidden, 'ldwork', 'info']


    if compq != 'N' and compq != 'I' and compq != 'U':
        raise SlycotParameterError('Parameter compq had an illegal value', -1)

    if compz != 'N' and compz != 'I' and compz != 'U':
        raise SlycotParameterError('Parameter compz had an illegal value', -2)

    if joba != 'N' and joba != 'R' and joba != 'T':
        raise SlycotParameterError('Parameter joba had an illegal value', -3)

    if ldwork is None:
        ldwork = max(1, n+p, min(l,n) + max(3*n-1, m, l))


    if compq == 'N' and compz == 'N':
        A,E,B,C,ranke,rnka22,info = _wrapper.tg01fd_nn(joba,l,n,m,p,A,E,B,C,tol,ldwork)
        Q = None
        Z = None
    elif compq == 'I' and compz == 'I':
        A,E,B,C,Q,Z,ranke,rnka22,info = _wrapper.tg01fd_ii(joba,l,n,m,p,A,E,B,C,tol,ldwork)
    elif compq == 'U' and compz == 'U':
        A,E,B,C,Q,Z,ranke,rnka22,info = _wrapper.tg01fd_uu(joba,l,n,m,p,A,E,B,C,Q,Z,tol,ldwork)
    else:
        raise NotImplementedError(
            "The combination of compq and compz is not implemented")

    raise_if_slycot_error(info, arg_list)

    if joba == 'N':
        rnka22 = None

    return A,E,B,C,ranke,rnka22,Q,Z

# to be replaced by python wrappers
